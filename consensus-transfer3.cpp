// Time-stamp: <2023-06-02 15:28:53 kobayashi>

// 合意制御シミュレーション C++ ver
// with 情報転送

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <climits>
#include <cmath>
#include <random>
#include <algorithm>
#include <vector>
#include <deque>
#include <unordered_set>
#include <memory>

using std::shared_ptr;
using std::vector;

std::mt19937 rnd;

////////////////////////////////////////////////////////////////

// パラメータ

const double XYRange = 150.0; // 領域のXY範囲(+/-)[m]
int NumAgent = 20;            // 領域内のエージェント(ロボット)の数

const double ConsensusRange = 1.0;           // 合意したと判定する半径[m]
const double Vmax = 30.0 * 1000 / (60 * 60); // ロボットの最大移動速度[m/s]（km/h = 1000/(60*60) m/s）
const bool RandomOffset = false;             // 動作開始時刻をずらす(1制御周期内)
double ControlInterval = 0.1;                // ロボットの制御周期[s]

const double CommnicationRange = 100.0; // 通信半径[m]
const int DataRate = 24;                // データレート[Mbps] //6,9,12,18,24,36,48,54 既定値
int DataLength = 32;                    // パケット長[byte]
int Payload = 2304;                     // 最大ペイロード長[byte]
int Transfer_DataLength = 128;          // 転送位置情報のデータ長//16,32,64,128[byte]
int CW = 1023;                          // 15(CWmin)-1023(CWmax) 既定値(2^W-1)

int NumSim = 100;             // シミュレーション回数
const double TimeSpan = 60.0; // 総実行時間[s]
const double TimeStep = 0.01; // 処理を行う時間間隔[s]

////////////////////////////////////////////////////////////////

struct LocationData
{
    int id;
    double x;
    double y;
    double location;
};

struct TransferData
{
    int id;
    double tilde_x;
    double tilde_y;
    // double z; not used
};

class Packet
{
public:
    int id;
    double x;
    double y;
    double tilde_x;
    double tilde_y;
    // double z; not used

    vector<TransferData> transfer; // 転送データのリスト

    Packet(const int id, const double x, const double y, const double tilde_x, const double tilde_y)
    {
        this->id = id;
        this->x = x;
        this->y = y;
        this->tilde_x = tilde_x;
        this->tilde_y = tilde_y;
    }

    int get_datalength()
    {
        // return headerlength + sizeof(id) + sizeof(x)*3; //8(UDP)+4(int)+8(double)*3=36
        int datalength = DataLength + Transfer_DataLength * this->transfer.size();

        // 最大ペイロード長を超えた場合、超えた分を送信することはできない
        // 転送位置情報から削る
        if (datalength > Payload)
        {
            for (auto &tra : this->transfer)
            {
                this->transfer.pop_back();
                datalength = DataLength + Transfer_DataLength * this->transfer.size();
                if (datalength < Payload)
                {
                    break;
                }
            }
        }

        return datalength;
    }
};

#include "CSMA.hpp"

class Agent
{
public:
    int id;
    double x;                             // エージェントが持つX座標の情報[m]
    double y;                             // エージェントが持つY座標の情報[m]
    vector<shared_ptr<Packet>> adjacency; // エージェントが持つ隣接エージェントの情報
    double ux;                            // エージェントが決定したX方向制御入力[m]
    double uy;                            // エージェントが決定したY方向制御入力[m]

    Agent(const int id, const double x, const double y)
    {
        this->id = id;
        this->x = x;
        this->y = y;
        this->ux = 0;
        this->uy = 0;
    }

    void decision(const double x, const double y, const vector<shared_ptr<Packet>> &adjacency)
    {
        // 隣接エージェントの位置情報に基づいて平均合意の制御を行う
        // 制御モデル： xi[k+1] = xi[k] + ui[k]，ui[k] = eps * Σ(xi[k] - xj[k])

        // 位置情報とともに転送されてきた位置情報を制御入力として用いる．ただし，以下を考慮する．
        // - 最新のxj[k]を受け取っているが別の隣接からの転送に1制御周期前のxj[k-1]が含まれている
        // - 隣接からの転送に自分が1制御周期前に送ったxi[k-1]が含まれている
        // - 複数の隣接からの転送に同じxj[k-1]が含まれている

        std::unordered_set<int> ids;
        vector<TransferData> transfer;

        ids.insert(this->id); // 自分のID
        for (auto &adj : adjacency)
            ids.insert(adj->id); // 隣接のID

        // 転送されてきた情報をまとめる
        for (auto &adj : adjacency)
        {
            for (auto &tra : adj->transfer)
            {
                if (ids.find(tra.id) == ids.end())
                {
                    ids.insert(tra.id);
                    transfer.push_back(tra);
                }
            }
        }

        double eps = 1.0 / (adjacency.size() + transfer.size() + 1); // eps<1/グラフの最大次数 らしい(いろいろある)
        double sumx = 0.0;
        double sumy = 0.0;
        for (auto &adj : adjacency)
        {
            sumx += x - adj->x;
            sumy += y - adj->y;
        }
        for (auto &tra : transfer)
        {
            sumx += x - tra.tilde_x;
            sumy += y - tra.tilde_y;
        }
        this->ux = -eps * sumx;
        this->uy = -eps * sumy;

        // エージェントが持つ情報を更新する（現時点では特に利用してない）
        // this->update(x, y, adjacency);
    }

    // エージェントが持つX座標，Y座標，隣接エージェントの情報を更新する
    void update(const double x, const double y, const vector<shared_ptr<Packet>> &adjacency)
    {
        this->x = x;
        this->y = y;
        this->adjacency = adjacency;
    }
};

struct History
{
    double t;           // 時刻
    double x;           // X座標
    double y;           // Y座標
    double vx;          // X速度
    double vy;          // Y速度
    int num_adjacency;  // 通信範囲内のロボット数
    int num_packets_rx; // 制御周期内に受信したパケット数
    CSMAInfo com_info;  // CSMAパケットカウント数
};

class Robot
{
public:
    int time_offset;         // ロボットの時間オフセット (0だと全員が同期)
    shared_ptr<Agent> agent; // ロボットの制御を行うエージェント
    shared_ptr<CSMA> com;    // ロボットの通信方式
    double x;                // ロボットのX座標[m]
    double y;                // ロボットのY座標[m]
    double vx;               // ロボットのX方向移動速度[m/s]
    double vy;               // ロボットのY方向移動速度[m/s]
    vector<History> history; // ロボットの履歴
    vector<LocationData> location;

    Robot(const shared_ptr<Agent> agent)
    {
        std::uniform_int_distribution<int> uniform(0, int(ControlInterval / TimeStep));
        this->com = std::make_shared<CSMA>(DataRate, CW);
        this->agent = agent;
        this->x = agent->x;
        this->y = agent->y;
        this->vx = 0.0;
        this->vy = 0.0;
        this->time_offset = (RandomOffset) ? uniform(rnd) : 0;
    }

    // ロボットが時刻インデックスtから１ステップ時間の間に実行する処理
    void one_step(const int t)
    {
        // 制御周期の整数倍かどうかを判定
        if ((t + this->time_offset) % int(ControlInterval / TimeStep) == 0)
        {
            // 制御周期のタイミングの場合

            // 履歴に追加
            // this->history.push_back({t*TimeStep, this->x, this->y, this->vx, this->vy, int(this->com->adjacency.size()), int(this->com->packets_rx.size()), this->com->info});

            // エージェントはロボットのX座標，Y座標，隣接エージェントを取得し，それに基づいて制御入力を決定する
            this->agent->decision(this->x, this->y, this->com->packets_rx);
            this->calc_velocity(this->agent->ux, this->agent->uy); // 速度を計算

            // 速度をもとに次周期における自己位置を予測
            // 現在の位置情報に速度と制御周期をかけたものを足して1制御周期後の位置を予測
            double tilde_x = this->x + (this->vx * ControlInterval);
            double tilde_y = this->y + (this->vy * ControlInterval);

            // 自身の位置情報＋次周期における自身の位置情報＋制御周期内に受け取った推定位置情報のパケットを送信し，受信パケットをクリア
            auto packet = std::make_shared<Packet>(this->agent->id, this->x, this->y, tilde_x, tilde_y);
            // 転送する位置情報は予測位置情報のみ
            // 自己位置との距離が最大の制御対象の位置情報から転送リストに追加する
            for (auto &packet_rx : this->com->packets_rx)
            {
                double distance = sqrt(pow(this->x - packet_rx->x, 2) + pow(this->y - packet_rx->y, 2));

                LocationData newLocation;
                newLocation.id = packet_rx->id;
                newLocation.x = packet_rx->x;
                newLocation.y = packet_rx->y;
                newLocation.location = distance;

                location.push_back(newLocation);

                double threshold = 50.0;
                if (distance < threshold)
                {
                    packet->transfer.push_back(TransferData{packet_rx->id, packet_rx->tilde_x, packet_rx->tilde_y});
                }
            }
            this->com->add_packet(packet);
            this->com->packets_rx.clear();
        }
    }

    // 制御入力は次の制御周期までの座標の変化量[m]であるため，変化量を時間で割って移動速度[m/s]を計算する
    void calc_velocity(const double ux, const double uy)
    {
        this->vx = ux / ControlInterval;
        this->vy = uy / ControlInterval;
        double v = std::hypot(vx, vy);
        if (v > Vmax)
        { // 最大移動速度を超えないという制限を設ける
            this->vx *= Vmax / v;
            this->vy *= Vmax / v;
        }
    }

    // ロボットが１ステップ時間だけ移動する
    void move()
    {
        this->x += this->vx * TimeStep; // 速度×時間
        this->y += this->vy * TimeStep; // 速度×時間
    }
};

class World
{
public:
    vector<shared_ptr<Robot>> robots; // ロボット

    World()
    {
    }

    // シミュレーションを実行する
    double run()
    {
        // ステップ数＝総実行時間/1ステップ時間
        int time_samples = int(TimeSpan / TimeStep);

        // 合意した時刻
        double time_consensus = 0;

        for (int t = 0; t < time_samples; t++)
        {
            this->one_step(t); // 1ステップ

            if (this->consensus_check())
            {
                time_consensus = (t + 1) * TimeStep;
                return time_consensus; // 合意した時点で終了
            }
        }
        return time_consensus;
    }

    // 時刻インデックスtから１ステップ時間の間に実行する処理
    void one_step(const int t)
    {
        // ロボットの接続状況を設定する
        this->set_adjacency();

        // 各ロボットについて１ステップ時間の処理をすすめる
        for (auto &robot : this->robots)
            robot->one_step(t); // 1ステップ

        // 各ロボットについて１ステップ時間の通信をすすめる
        int s = 0;
        int slot_samples = std::ceil(TimeStep / CSMA::SlotTime); // １ステップ時間内の通信スロット数
        while (s < slot_samples)
        {
            // 送信状態を更新する（全員事前に行う必要がある）
            for (auto &robot : this->robots)
                robot->com->update_tx();
            // 受信状態を更新する（全員事前に行う必要がある）
            for (auto &robot : this->robots)
                robot->com->update_rx();

            // 状態が変わらない間はスキップして高速化
            int skip = 1;
            int min_counter = INT_MAX;
            for (auto &robot : this->robots)
            {
                int next_state = robot->com->next_state;
                // one_slot()において状態が変わるのは以下の3通り．動作カウントの最小値を取得する．
                if (((next_state == CSMAState::Backoff) || (next_state == CSMAState::DIFS) || (next_state == CSMAState::Tx)))
                {
                    if (min_counter > robot->com->counter)
                        min_counter = robot->com->counter;
                }
            }
            if (min_counter != INT_MAX)
            {
                skip = min_counter;
                if (s + skip > slot_samples)
                    skip = slot_samples - s; // 残り時間を超えてスキップできないので制限
            }

            // 各ロボットについての通信を１スロット時間だけ処理をすすめる
            for (auto &robot : this->robots)
                robot->com->one_slot(skip);

            // 全員アイドル状態ならbreakして高速化
            bool idle_all = true;
            for (auto &robot : this->robots)
            {
                if (robot->com->state != CSMAState::Idle)
                {
                    idle_all = false;
                    break;
                }
            }
            if (idle_all)
                break;

            // スロットを進める
            s += skip;
        }

        // 各ロボットについて１ステップ時間だけ移動する
        for (auto &robot : this->robots)
            robot->move();
    }

    // ロボットの接続状況を設定する
    void set_adjacency()
    {
        // 初期化
        for (auto &robot : this->robots)
            robot->com->adjacency.clear();

        // すべてのロボット同士の組み合わせについて処理を行う
        for (int i = 0; i < this->robots.size(); i++)
        {
            for (int j = i + 1; j < this->robots.size(); j++)
            {
                // ロボット同士の距離を算出
                double distance = std::hypot(this->robots[i]->x - this->robots[j]->x, this->robots[i]->y - this->robots[j]->y);
                // 通信範囲内であれば通信対象として追加する
                if (distance <= CommnicationRange)
                {
                    this->robots[i]->com->adjacency.push_back(this->robots[j]->com);
                    this->robots[j]->com->adjacency.push_back(this->robots[i]->com);
                }
            }
        }
    }

    // 合意判定
    bool consensus_check()
    {
        // すべてのロボット同士の組み合わせについて処理を行う
        for (int i = 0; i < this->robots.size(); i++)
        {
            for (int j = i + 1; j < this->robots.size(); j++)
            {
                // ロボット同士の距離を算出
                double distance = std::hypot(this->robots[i]->x - this->robots[j]->x, this->robots[i]->y - this->robots[j]->y);
                if (distance > ConsensusRange)
                    return false; // 1台でも範囲外ならFalse
            }
        }
        return true;
    }
};

int main(int argc, char **argv)
{
    // std::random_device rnd_dev;
    // rnd.seed(rnd_dev());

    if (argc > 1)
        NumAgent = atoi(argv[1]);
    if (argc > 2)
        ControlInterval = atof(argv[2]);
    if (argc > 3)
        DataLength = atoi(argv[3]);
    if (argc > 4)
        CW = atoi(argv[4]);
    if (argc > 5)
        NumSim = atoi(argv[5]);

    printf("# 状態情報の選別を行う情報転送手段プログラム\n");
    printf("# \n");
    printf("# XYRange = +/-%.0lf [m]\n", XYRange);
    printf("# NumAgent = %d \n", NumAgent);
    printf("# ConsensusRange = %.0lf [m]\n", ConsensusRange);
    printf("# Vmax = %.3lf [m/s]\n", Vmax);
    printf("# RandomOffset = %d \n", RandomOffset);
    printf("# ControlInterval = %.3lf [s]\n", ControlInterval);
    printf("# CommnicationRange = %.0lf [m]\n", CommnicationRange);
    printf("# DataRate = %d [Mbps]\n", DataRate);
    printf("# DataLength = %d [byte]\n", DataLength);
    printf("# CW = %d \n", CW);
    printf("# NumSim = %d \n", NumSim);
    printf("# TimeSpan = %.0lf [s]\n", TimeSpan);
    printf("# TimeStep = %.3lf [s]\n", TimeStep);
    printf("# \n");

    int simulation_count = 0;
    int consensus_count = 0;
    double consensus_time = 0;

    int64_t count_tx_packet = 0;
    int64_t count_rx_packet = 0;
    int64_t count_tx_discard = 0;
    int64_t count_rx_collision = 0;

    for (int loop = 0; simulation_count < NumSim; loop++)
    {
        rnd.seed(loop); // シードを固定すると同じ乱数が発生する(配置を同じにするため)

        World world;

        std::uniform_real_distribution<double> uniform(-XYRange, XYRange);

        // 領域内に一様ランダム配置
        for (int i = 0; i < NumAgent; i++)
        {
            double x = uniform(rnd);
            double y = uniform(rnd);
            auto agent = std::make_shared<Agent>(i, x, y);
            auto robot = std::make_shared<Robot>(agent);
            world.robots.push_back(robot);
        }

        // ロボットの接続状況をチェックする（隣接を設定する）
        world.set_adjacency();

        // 接続されていないロボットがいればその配置はスキップ
        bool adjacency_check = true;
        for (auto &robot : world.robots)
        {
            if (robot->com->adjacency.size() == 0)
            {
                adjacency_check = false;
                break;
            }
        }
        if (!adjacency_check)
            continue;

        // シミュレーション
        double time_consensus = world.run();

        // 合意してたら時間は0以上が返ってくる
        if (time_consensus > 0)
        {
            consensus_count += 1;
            consensus_time += time_consensus;
        }

        // 通信パケットのカウント
        for (auto &robot : world.robots)
        {
            count_tx_packet += robot->com->info.count_tx_packet;
            count_rx_packet += robot->com->info.count_rx_packet;
            count_tx_discard += robot->com->info.count_tx_discard;
            count_rx_collision += robot->com->info.count_rx_collision;
        }

        simulation_count += 1;
    }

    printf("# ConsensusRate  ConsensusTime  DiscardRate  CollisionRate\n");

    printf("%.10lf(%d/%d)  %3.7lf  %.10lf  %.10lf \n",
           (double)consensus_count / simulation_count, consensus_count, simulation_count,
           (double)consensus_time / consensus_count,
           (double)count_tx_discard / count_tx_packet,
           (double)count_rx_collision / count_rx_packet);
}
