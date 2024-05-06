// Time-stamp: <2023-10-19 11:02:47 kobayashi>

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

using std::vector;
using std::shared_ptr;

std::mt19937 rnd;


////////////////////////////////////////////////////////////////

// パラメータ

const double XYRange = 150.0; //領域のXY範囲(+/-)[m]
int NumAgent = 30; //領域内のエージェント(ロボット)の数

const double ConvergenceRange = 1.0; //合意したと判定する半径[m]
const double Vmax = 30.0 * 1000/(60*60); //ロボットの最大移動速度[m/s]（km/h = 1000/(60*60) m/s）
const bool RandomOffset = false; //動作開始時刻をずらす(1制御周期内)
double ControlInterval = 0.1; //ロボットの制御周期[s]

const double CommnicationRange = 100.0;  //通信半径[m]
const int DataRate = 6; //データレート[Mbps] //6,9,12,18,24,36,48,54 既定値
int DataLength = 64; //パケット長[byte]
int CW = 1023; //15(CWmin)-1023(CWmax) 既定値(2^W-1)

int NumSim = 100; //シミュレーション回数
const double TimeSpan = 60.0; //総実行時間[s]
const double TimeStep = 0.01; //処理を行う時間間隔[s]

////////////////////////////////////////////////////////////////


struct TransferData {
int id;
double x;
double y;
//double z; not used
};

class Packet {
public:
int id;
double x;
double y;
//double z; not used

vector<TransferData> transfer; //転送データのリスト

Packet(const int id, const double x, const double y) {
this->id = id;
this->x = x;
this->y = y;
}

int get_datalength() {
//return headerlength + sizeof(id) + sizeof(x)*3; //8(UDP)+4(int)+8(double)*3=36
return DataLength * (1 + this->transfer.size());
}
};


#include "CSMA.hpp"


class Agent {
public:
int id;
double x; //エージェントが持つX座標の情報[m]
double y; //エージェントが持つY座標の情報[m]
double ux; // エージェントが決定したX方向制御入力[m]
double uy; // エージェントが決定したY方向制御入力[m]
vector<shared_ptr<Packet>> adjacency; //エージェントが持つ隣接エージェントの情報
vector<TransferData> transfer; //エージェントが持つ転送されてきたエージェントの情報


Agent(const int id, const double x, const double y) {
this->id = id;
this->x = x;  
this->y = y; 
this->ux = 0;
this->uy = 0;
}

void decision(const double x, const double y, const vector<shared_ptr<Packet>> &adjacency) {
// 隣接エージェントの位置情報に基づいて平均合意の制御を行う
// 制御モデル： xi[k+1] = xi[k] + ui[k]，ui[k] = eps * Σ(xi[k] - xj[k])

// 位置情報とともに転送されてきた位置情報を制御入力として用いる．ただし，以下を考慮する．
// - 最新のxj[k]を受け取っているが別の隣接からの転送に1制御周期前のxj[k-1]が含まれている
// - 隣接からの転送に自分が1制御周期前に送ったxi[k-1]が含まれている
// - 複数の隣接からの転送に同じxj[k-1]が含まれている

vector<TransferData> transfer;

std::unordered_set<int> ids = {this->id}; //自分のID
for (auto &adj : adjacency) ids.insert(adj->id); //隣接のID

// 転送されてきた情報をまとめる
for (auto &adj : adjacency) {
    for (auto &tra : adj->transfer) {
    if (ids.find(tra.id) == ids.end()) {
        ids.insert(tra.id);
        transfer.push_back(tra);
    }
    }
}

double eps = 1.0/(adjacency.size()+transfer.size()+1); // eps<1/グラフの最大次数 らしい(いろいろある)
double sumx = 0.0;
double sumy = 0.0;
for (auto &adj : adjacency) {
    sumx += x - adj->x;
    sumy += y - adj->y;
}
for (auto &tra : transfer) {
    sumx += x - tra.x;
    sumy += y - tra.y;
}
this->ux = -eps * sumx;
this->uy = -eps * sumy;

// エージェントが持つ情報を更新する
this->update(x, y, adjacency, transfer);
}

// エージェントが持つX座標，Y座標，隣接エージェントの情報を更新する
void update(const double x, const double y, const vector<shared_ptr<Packet>> &adjacency, const vector<TransferData> &transfer) {
this->x = x;
this->y = y;
this->adjacency = adjacency;
this->transfer = transfer;
}
};



struct Info {
int count_control;      //制御回数
int count_adjacency;    //通信範囲内のロボット数
int count_adjacency_rx; //制御周期内に受信した位置情報の数
int count_transfer_rx;  //制御周期内に受信した転送位置情報の数（重複カット後）  
};

struct History {
double t;  //時刻
double x;  //X座標
double y;  //Y座標
double vx; //X速度
double vy; //Y速度
};


class Robot {
public:
int time_offset; // ロボットの時間オフセット (0だと全員が同期)
shared_ptr<Agent> agent; // ロボットの制御を行うエージェント
shared_ptr<CSMA> com; // ロボットの通信方式
double x; // ロボットのX座標[m]
double y; // ロボットのY座標[m]
double vx; // ロボットのX方向移動速度[m/s]
double vy; // ロボットのY方向移動速度[m/s]
vector<History> history;  // ロボットの履歴
Info info;

Robot(const shared_ptr<Agent> agent) {
std::uniform_int_distribution<int> uniform(0, int(ControlInterval/TimeStep));
this->com = std::make_shared<CSMA>(DataRate, CW);
this->agent = agent;
this->x = agent->x;
this->y = agent->y;
this->vx = 0.0;
this->vy = 0.0;
this->time_offset = (RandomOffset)? uniform(rnd) : 0;
this->info = {0};
}

// ロボットが時刻インデックスtから１ステップ時間の間に実行する処理
void one_step(const int t) {
// 制御周期の整数倍かどうかを判定
if ((t + this->time_offset) % int(ControlInterval/TimeStep) == 0) {
    // 制御周期のタイミングの場合
    
    // エージェントはロボットのX座標，Y座標，隣接エージェントを取得し，それに基づいて制御入力を決定する
    this->agent->decision(this->x, this->y, this->com->packets_rx);
    // 速度を計算
    this->calc_velocity(this->agent->ux, this->agent->uy); 
    
    // 履歴に追加
    //this->history.push_back({t*TimeStep, this->x, this->y, this->vx, this->vy});
    this->info.count_control++;
    this->info.count_adjacency += int(this->com->adjacency.size());
    this->info.count_adjacency_rx += int(this->agent->adjacency.size());
    this->info.count_transfer_rx  += int(this->agent->transfer.size());
    
    // 自身の位置情報＋制御周期内に受け取った位置情報のパケットを送信し，受信パケットをクリア
    auto packet = std::make_shared<Packet>(this->agent->id, this->x, this->y);
    for (auto &packet_rx : this->com->packets_rx) {
    packet->transfer.push_back(TransferData{packet_rx->id, packet_rx->x, packet_rx->y});
    }
    this->com->add_packet(packet);
    this->com->packets_rx.clear();
}
}

// 制御入力は次の制御周期までの座標の変化量[m]であるため，変化量を時間で割って移動速度[m/s]を計算する
void calc_velocity(const double ux, const double uy) {
this->vx = ux/ControlInterval;
this->vy = uy/ControlInterval;
double v  = std::hypot(vx, vy);
if (v > Vmax) { // 最大移動速度を超えないという制限を設ける
    this->vx *= Vmax/v;
    this->vy *= Vmax/v;
}
}

// ロボットが１ステップ時間だけ移動する
void move() {
this->x += this->vx * TimeStep; // 速度×時間
this->y += this->vy * TimeStep; // 速度×時間
}
};


class World {
public:
vector<shared_ptr<Robot>> robots; // ロボット

World() {

}

// シミュレーションを実行する
double run() {
// ステップ数＝総実行時間/1ステップ時間
int time_samples = int(TimeSpan/TimeStep);

// 合意した時刻
double time_convergence = 0;

for (int t=0; t < time_samples; t++) {
    this->one_step(t); // 1ステップ
    
    if (this->convergence_check()) {
    time_convergence = (t+1)*TimeStep;
    return time_convergence; //合意した時点で終了
    }
}
return time_convergence;
}

// 時刻インデックスtから１ステップ時間の間に実行する処理
void one_step(const int t) {
// ロボットの接続状況を設定する
this->set_adjacency();

// 各ロボットについて１ステップ時間の処理をすすめる
for (auto &robot : this->robots) robot->one_step(t); // 1ステップ

// 各ロボットについて１ステップ時間の通信をすすめる
int s = 0;
int slot_samples = std::ceil(TimeStep/CSMA::SlotTime); //１ステップ時間内の通信スロット数
while (s < slot_samples) {
    // 送信状態を更新する（全員事前に行う必要がある）
    for (auto &robot : this->robots) robot->com->update_tx();
    // 受信状態を更新する（全員事前に行う必要がある）
    for (auto &robot : this->robots) robot->com->update_rx();
    
    // 状態が変わらない間はスキップして高速化
    int skip = 1;
    int min_counter = INT_MAX;
    for (auto &robot : this->robots) {
    int next_state = robot->com->next_state;
    // one_slot()において状態が変わるのは以下の3通り．動作カウントの最小値を取得する．
    if (((next_state == CSMAState::Backoff) || (next_state == CSMAState::DIFS) || (next_state == CSMAState::Tx))) {
        if (min_counter > robot->com->counter) min_counter = robot->com->counter;
    }
    }
    if (min_counter != INT_MAX) {
    skip = min_counter;
    if (s+skip > slot_samples) skip = slot_samples-s; //残り時間を超えてスキップできないので制限
    }
    
    // 各ロボットについての通信を１スロット時間だけ処理をすすめる
    for (auto &robot : this->robots) robot->com->one_slot(skip);
    
    // 全員アイドル状態ならbreakして高速化
    bool idle_all = true;
    for (auto &robot : this->robots) {
    if (robot->com->state != CSMAState::Idle) {
        idle_all = false;
        break;
    }
    }
    if (idle_all) break;
    
    // スロットを進める
    s += skip;
}

// 各ロボットについて１ステップ時間だけ移動する
for (auto &robot : this->robots) robot->move();
}

// ロボットの接続状況を設定する
void set_adjacency() {
// 初期化
for (auto &robot : this->robots) robot->com->adjacency.clear();

// すべてのロボット同士の組み合わせについて処理を行う
for (int i=0; i < this->robots.size(); i++) {
    for (int j=i+1; j < this->robots.size(); j++) {
    // ロボット同士の距離を算出
    double distance = std::hypot(this->robots[i]->x - this->robots[j]->x, this->robots[i]->y - this->robots[j]->y);
    // 通信範囲内であれば通信対象として追加する
    if (distance <= CommnicationRange) {
        this->robots[i]->com->adjacency.push_back(this->robots[j]->com);
        this->robots[j]->com->adjacency.push_back(this->robots[i]->com);
    }
    }
}
}

// 合意判定
bool convergence_check() {
// すべてのロボット同士の組み合わせについて処理を行う
for (int i=0; i < this->robots.size(); i++) {
    for (int j=i+1; j < this->robots.size(); j++) {
    // ロボット同士の距離を算出
    double distance = std::hypot(this->robots[i]->x - this->robots[j]->x, this->robots[i]->y - this->robots[j]->y);
    if (distance > ConvergenceRange) return false; // 1台でも範囲外ならFalse
    }
}
return true;
}

};



int main(int argc, char **argv) {
//std::random_device rnd_dev;
//rnd.seed(rnd_dev());

if (argc > 1) NumAgent = atoi(argv[1]);
if (argc > 2) ControlInterval = atof(argv[2]);
if (argc > 3) DataLength = atoi(argv[3]);
if (argc > 4) CW = atoi(argv[4]);
if (argc > 5) NumSim = atoi(argv[5]);

printf("# 状態情報の予測を行わない情報転送手段プログラム\n");
printf("# \n");
printf("# XYRange = +/-%.0lf [m]\n", XYRange);  
printf("# NumAgent = %d \n", NumAgent);
printf("# ConvergenceRange = %.0lf [m]\n", ConvergenceRange);
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
int convergence_count = 0;
double convergence_time = 0;

int64_t count_tx_packet = 0;  
int64_t count_rx_packet = 0;  
int64_t count_tx_discard = 0;  
int64_t count_rx_collision = 0;  

int64_t count_control = 0;
int64_t count_adjacency = 0;
int64_t count_adjacency_rx = 0;
int64_t count_transfer_rx = 0;


for (int loop=0; loop < NumSim; loop++) {
rnd.seed(loop); //シードを固定すると同じ乱数が発生する(配置を同じにするため)

World world;

std::uniform_real_distribution<double> uniform(-XYRange, XYRange);

//領域内に一様ランダム配置
for (int i=0; i < NumAgent; i++) {
    double x = uniform(rnd);
    double y = uniform(rnd);
    auto agent = std::make_shared<Agent>(i, x, y);
    auto robot = std::make_shared<Robot>(agent);
    world.robots.push_back(robot);
}

//ロボットの接続状況をチェックする（隣接を設定する）
world.set_adjacency();

//接続されていないロボットがいればその配置はスキップ
bool adjacency_check = true;
for (auto &robot : world.robots) {
    if (robot->com->adjacency.size() == 0) { 
    adjacency_check = false;
    break;
    }
}
if (!adjacency_check) continue; 

//シミュレーション
double time_convergence = world.run();

//合意してたら時間は0以上が返ってくる
if (time_convergence > 0) {
    convergence_count += 1;
    convergence_time += time_convergence;
}


//通信パケット等のカウント
for (auto &robot : world.robots) {
    count_tx_packet += robot->com->info.count_tx_packet;
    count_rx_packet += robot->com->info.count_rx_packet;
    count_tx_discard += robot->com->info.count_tx_discard;
    count_rx_collision += robot->com->info.count_rx_collision;
    
    count_control += robot->info.count_control;
    count_adjacency += robot->info.count_adjacency;
    count_adjacency_rx += robot->info.count_adjacency_rx; 
    count_transfer_rx += robot->info.count_transfer_rx; 
}

simulation_count += 1;
}

printf("# ConvergenceRate  ConvergenceTime  DiscardRate  CollisionRate  AvgNumAdj  AvgNumAdjRx  AvgNumTraRx\n");

printf("%.10lf  %3.7lf  %.10lf  %.10lf  %.10lf  %.10lf  %.10lf\n",
        (double)convergence_count/simulation_count,
        (double)convergence_time/convergence_count,
        (double)count_tx_discard/count_tx_packet,
        (double)count_rx_collision/count_rx_packet,
        (double)count_adjacency/count_control,
        (double)count_adjacency_rx/count_control,
        (double)count_transfer_rx/count_control);
}
