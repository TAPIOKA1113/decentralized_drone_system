// Time-stamp: <2023-10-19 10:45:44 kobayashi>
 
// CSMA(IEEE 802.11a/n)ヘッダファイル

/*
  IEEE 802.11a/n (5GHz)
  -------------------------------BPSK CodeRate 1/2
  Preamble 16 microsec
  PLCP header 4 microsec
  -------------------------------DataRate indicated in the PLCP header
  PLCP header 2 oct
  Payload             Ack
  - mac 24 oct        2+2+6+4=14 oct      
  - body 0-2312
  - fcs 4 oct
  Tail 6 bits
  Padding x bits with zero (at least 6 bits) such that the length of DATA field is a multiple of NDBPS(=4*DataRate)
  -------------------------------
  DataRate  Modulation  CodeRate  CodedBit/Subcarrier(NBPSC)  CodedBit/OFDM Symbol(NCBPS)  DataBit/OFDM Symbol(NDBPS)
  6 Mbps    BPSK        1/2       1                           48                           24
  9 Mbps    BPSK        3/4       1                           48                           36
  12Mbps    QPSK        1/2       2                           96                           48
  18Mbps    QPSK        3/4       2                           96                           72
  24Mbps    16-QAM      1/2       4                           192                          96
  27Mbps    16-QAM      9/16      4                           192                          108
  36Mbps    16-QAM      3/4       4                           192                          144
  48Mbps    64-QAM      2/3       6                           288                          192
  54Mbps    64-QAM      3/4       6                           288                          216
*/

#include <deque>
#include <memory>
#include <vector>

using std::vector;
using std::shared_ptr;


struct CSMAState {
  static constexpr int Idle = 0;
  static constexpr int Backoff = 1;
  static constexpr int Tx = 2;
  static constexpr int Rx = 3;
  static constexpr int DIFS = 4;
};

struct CSMAInfo {
  int count_tx_packet;    //送信しようとしたパケット数(破棄されたものも含む)
  int count_rx_packet;    //受信しようとしたパケット数(衝突したものも含む)
  int count_tx_discard;   //次のパケットが来たことで送信キューから破棄されたパケット数
  int count_rx_collision; //隣接が同時に送信したことで受信に失敗(衝突)したパケット数
  int count_tx_repeat;    //繰り返し送信しようとしたパケット数(count_tx_packetはこれも含む)
};


class CSMA {
public:
  //通信パラメータ
  static constexpr double SlotTime = 1e-6; //1μs 既定値
  static constexpr int BackoffSlot = 9; //μs 既定値
  static constexpr int DIFSSlot = 34; //μs 既定値
  double DataRate; //Mbps //6,9,12,18,24,36,48,54 既定値(設定可)
  int CW; //15(CWmin)-1023(CWmax) 既定値(設定可)
  
  std::uniform_int_distribution<int> uniform;
  
  int state;      //現在の状態
  int next_state; //次の状態
  int counter; //次の動作までのカウンタ(スロット数)
  int resume;  //バックオフが中断された場合にカウンタを保存
  int repeat;  //送信終了後に同じパケットの送信を繰り返す
  std::deque<shared_ptr<Packet>> packets; //送信パケットのキュー
  vector<shared_ptr<Packet>> packets_rx;  //受信パケットのリスト
  
  vector<shared_ptr<CSMA>> adjacency; //隣接通信者のリスト(自身は知ることはできない)
  
  CSMAInfo info;
  
  
  CSMA(const double datarate, const int cw, const int rep=0) {
    this->DataRate = datarate;
    this->CW = cw;
    this->uniform = std::uniform_int_distribution<int>(0, cw);
    this->state = CSMAState::Idle;
    this->next_state = CSMAState::Idle;
    this->counter = 0;
    this->resume = -1;
    this->repeat = rep;
    this->info = {0};
  }
  
  int get_dataslot(const shared_ptr<Packet> packet) {
    return 16+4+std::ceil(((2+24+packet->get_datalength()+4)*8+6)/(4*this->DataRate))*4; //μs 既定値
  }
  
  // パケットを送信キューに追加する（ここでは1個のパケットしか送信待ちにしないと仮定）
  void add_packet(const shared_ptr<Packet> packet) {
    // 送信中でなければパケットを破棄
    if ((this->state != CSMAState::Tx) && (this->packets.size() != 0)) {
      this->packets.pop_front();
      this->info.count_tx_discard++;
    }
    // パケットを追加
    this->packets.push_back(packet);
    this->info.count_tx_packet++;
  }
  
  // すべての通信は同時刻に並列して行われすはずだが，プログラム上では前から順番に処理するしかない
  // それゆえ後で処理する通信状態が前で処理する時点で分からない（通信を検知できない）問題が生じる
  // そのため，まずはすべての通信状態を更新してから順番に通信処理をするという２度手間で行う
  
  void update_tx() {
    // 次の状態はアイドルだが，パケットが生じた場合はバックオフ処理へ更新する（後のif文にも入る）
    if (this->next_state == CSMAState::Idle) {
      if (this->packets.size() != 0) {
        this->counter = CSMA::BackoffSlot * uniform(rnd);
        this->resume = -1;      
        this->next_state = CSMAState::Backoff;
      } else {
        return;
      }
    }
    
    // 次の状態はバックオフだが，隣接が送信しておらずバックオフカウンタが0の場合，送信処理へ更新する
    if (this->next_state == CSMAState::Backoff) {
      for (auto &adj : this->adjacency) {
        if (adj->state == CSMAState::Tx) return;
      }
      if (this->counter == 0) {
        this->counter = this->get_dataslot(this->packets.front());
        this->next_state = CSMAState::Tx;
      }
      return;
    }
    
    // 送信が終了し，次のDIFS状態になる場合，パケットを送信済みとして削除する
    if ((this->state == CSMAState::Tx) && (this->next_state == CSMAState::DIFS)) {
      if (this->repeat && (this->packets.size() == 1)) {
        //繰り返し送信を行う場合(ただし新しいパケットがない場合)
        add_packet(this->packets.front());
        this->info.count_tx_repeat++;
      }
      this->packets.pop_front();
    }
  }
  
  void update_rx() {
    // 送信している隣接の通信者を取得する
    vector<shared_ptr<CSMA>> adjacency_tx;
    for (auto &adj : this->adjacency) {
      if (adj->next_state == CSMAState::Tx) adjacency_tx.push_back(adj);
    }
    
    // 次の状態がアイドル，DIFS，バックオフの場合に，
    // 隣接が送信をしていた場合，受信処理へ更新する（後のif文にも入る）
    if ((this->next_state == CSMAState::Idle) || (this->next_state == CSMAState::DIFS)) {
      if (adjacency_tx.size() != 0) {
        this->next_state = CSMAState::Rx;
      } else {
        return;
      }
    }
    else if (this->next_state == CSMAState::Backoff) {
      if (adjacency_tx.size() != 0) {
        this->resume = this->counter; //バックオフを中断(カウンタを保存)
        this->next_state = CSMAState::Rx;
      } else {
        return;
      }
    }  
    
    // 受信状態において
    if (this->next_state == CSMAState::Rx) {
      if (adjacency_tx.size() == 0) {
        // 隣接が1台も送信していない場合，DIFS処理へ更新する
        this->counter = CSMA::DIFSSlot;
        this->next_state = CSMAState::DIFS;
      }
      else if (adjacency_tx.size() == 1) {
        // 隣接が1台のみ送信していた場合，パケットを受信（送信開始時点でパケットを受信と仮定）
        auto adj_tx = adjacency_tx[0];
        if (adj_tx->counter == this->get_dataslot(adj_tx->packets.front())) { //送信開始時点
          this->packets_rx.push_back(adj_tx->packets.front());
          this->info.count_rx_packet++;
        }
      }
      else {
        // 隣接が複数台送信していた場合，パケット衝突が発生するので送信されたパケットは受信不可能（受信パケットリストから削除）
        for (auto &adj_tx : adjacency_tx) {
          auto position = std::find(this->packets_rx.begin(), this->packets_rx.end(), adj_tx->packets.front());
          if (position != this->packets_rx.end()) {
            this->packets_rx.erase(position);
            this->info.count_rx_collision++;
          }
        }
      }
    }
  }
  
  void one_slot(const int skip=1) {
    // 次の状態に遷移する
    this->state = this->next_state;
    
    // バックオフ状態において
    if (this->state == CSMAState::Backoff) {
      this->counter -= skip;
      if (this->counter == 0) {
        // バックオフカウンタが0になる場合，次は送信状態
        this->counter = this->get_dataslot(this->packets.front());
        this->next_state = CSMAState::Tx;
      }
      return;
    }
    
    // DIFS状態において
    if (this->state == CSMAState::DIFS) {
      this->counter -= skip;
      if (this->counter == 0) {
        if (this->packets.size() != 0) {
          // DIFSが終了し，パケットがある場合，次はバックオフ状態
          this->counter = (this->resume >= 0)? this->resume : CSMA::BackoffSlot * uniform(rnd);
          this->resume = -1;
          this->next_state = CSMAState::Backoff;
        } else {
          // DIFSが終了し，パケットがない場合，次はアイドル状態
          this->next_state = CSMAState::Idle;
        }
      }
      return;
    }
    
    // 送信状態において
    if (this->state == CSMAState::Tx) {
      this->counter -= skip;
      if (this->counter == 0) {
        // 送信終了する場合，次はDIFS
        this->counter = CSMA::DIFSSlot;
        this->next_state = CSMAState::DIFS;
      }
      return;
    }
  }
};
