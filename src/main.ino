#include <M5Unified.h>

#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <freertos/semphr.h>

//! INA226の設定アドレス
#define ENCODER_ADDR      (0x42)

//! 使用するI2Cのデータ線に割り当てるGPIOの番号
#define SDA               (0)

//! 使用するI2Cのクロック線に割り当てるGPIOの番号
#define SCL               (26)

//! I2Cのクロックの設定周波数
#define I2C_FREQ          (250000)

//! 液晶の明るさ(255までの数値)
#define BRIGHTNESS        (96)

//! ブザーの音量(255までの数値)
#define BUZZER_VOLUME     (240)

//! カウントアップタイマーの起動周期(msec)
#define TIMER_INTERVAL    (1000)

//! バッテリー残量取得タイマーの起動周期(msec)
#define BATTERY_INTERVAL  (30000)

//! UIルーパーのインターバル(msec)
#define LOOP_INTERVAL     (50)

//! 動作状態の識別子(待機状態)
#define STATE_READY       (1)

//! 動作状態の識別子(カウントダウン中)
#define STATE_COUNTDOWN   (2)

//! 動作状態の識別子(タイムアウト)
#define STATE_TIMEDOUT    (3)

//! ノブによるタイムアウト時間の変更対象(分の桁を変更)
#define STEP_MODE_MIN     (1)

//! ノブによるタイムアウト時間の変更対象(秒の10の桁を変更)
#define STEP_MODE_10SEC   (2)

//! ノブによるタイムアウト時間の変更対象(秒の1の桁を変更)
#define STEP_MODE_1SEC    (3)

//! サウンドモード(すべての音を再生)
#define SOUND_MODE1       (1)

//! サウンドアラームモード(経過音を再生しない)
#define SOUND_MODE2       (2)

//! サウンドアラームモード(すべての音を再生しない)
#define SOUND_MODE3       (3)

//! タイマーハンドラ（カウントアップタイマー用）
static TimerHandle_t timer1 = NULL;

//! タイマーハンドラ（バッテリー残量取得タイマー用）
static TimerHandle_t timer2 = NULL;

//! 排他制御用のミューテックス
static SemaphoreHandle_t mutex = NULL;

//! データを格納するための構造体
typedef struct {
  //! タイマカウント (秒数)
  int count;

  //! バッテリーレベル (パーセンテージ)
  int battery;

  //! 更新フラグ
  bool updated;
} data_set_t;

//! 液晶描画用のオフスクリーンウィンドウ
static M5Canvas canvas(&M5.Display);

//! マスターデータ (このデータは異なるコンテキストで共有)
static data_set_t master;

//! タイムアウト時間の変更が行われたか否かを表すフラグ
static bool isChange;

//! カウント値復帰用のバックアップ
static int backup;

//! 動作状態を格納する変数
static int state;

//! ノブによるタイムアウト時間の変更対象を格納する変数
static int stepMode;

//! サウンドモードを格納する変数
static int soundMode;

//! 音符マークのビットマップ (白, 16x16)
static const uint16_t note[256] = {
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xffff, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xffff, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xffff, 0xffff, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xffff, 0xffff, 0xffff, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xffff, 0x07e0, 0xffff, 0xffff, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xffff, 0x07e0, 0x07e0, 0xffff, 0xffff, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xffff, 0x07e0, 0x07e0, 0x07e0, 0xffff, 0xffff, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xffff, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0xffff, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xffff, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0xffff, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xffff, 0x07e0, 0x07e0, 0x07e0, 0xffff, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xffff, 0x07e0, 0x07e0, 0xffff, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xffff, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0xffff, 0xffff, 0xffff, 0xffff,
  0xffff, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
  0xffff, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0xffff, 0xffff, 0xffff, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
};

//! 音符マークのビットマップ (青, 16x16)
static const uint16_t note2[256] = {
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xf800, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xf800, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xf800, 0xf800, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xf800, 0xf800, 0xf800, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xf800, 0x07e0, 0xf800, 0xf800, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xf800, 0x07e0, 0x07e0, 0xf800, 0xf800, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xf800, 0x07e0, 0x07e0, 0x07e0, 0xf800, 0xf800, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xf800, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0xf800, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xf800, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0xf800, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xf800, 0x07e0, 0x07e0, 0x07e0, 0xf800, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xf800, 0x07e0, 0x07e0, 0xf800, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0xf800, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0xf800, 0xf800, 0xf800, 0xf800,
  0xf800, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0xf800, 0xf800, 0xf800, 0xf800, 0xf800,
  0xf800, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0xf800, 0xf800, 0xf800, 0xf800, 0xf800,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0xf800, 0xf800, 0xf800, 0x07e0,
  0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0, 0x07e0,
};


/**
 * ロータリーエンコーダのカウンタリセット処理
 */
static void
reset_encoder()
{
  const uint8_t reg40 = 1;
  M5.Ex_I2C.writeRegister(ENCODER_ADDR, 0x40, &reg40, 1, I2C_FREQ);
}

/**
 * ロータリーエンコーダの状態取得
 *
 * @param [out] dst1  ノブの回転量の書き込み先
 * @param [out] dst2  ボタンの押下状態の書き込み先
 *
 * @remark
 *  引数dst1で指定された領域には、ノブの回転量が書き込まれる。書き込まれる回転
 *  量は直近のreset_encoder()または本関数の呼び出し時からのカウント数になる。
 *  また、ボタン押下状態についてはレベルではなくエッジを返す（直近の本関数呼び
 *  出しから、開放→押下と変化したときにtrueを書き込む）。 
 *
 * @warning
 *  本関数が処理対象としているロータリーエンコーダーにおいて、ノッチと検出回転
 *  量の不一致が見られた。このため本関数にはその対応が組み込まれている。具体的
 *  にはノブを1ノッチ回転させた場合に、回転量が2つ検出される場合が多いため、実
 *  際に検出した回転量の1/2を返している。
 */
static bool
read_encoder_state(int* dst1, bool* dst2)
{
  bool ret;
  uint8_t reg00[4];
  uint8_t reg20;
  const uint8_t reg40 = 1;
  int knob;
  bool btn;
  static uint8_t prev = 1; // 前回のボタンレジスタの値

  do {
    ret = false;

    /*
     * ノブの積算回転量の読み出し
     */
    if (!M5.Ex_I2C.readRegister(ENCODER_ADDR, 0x00, reg00, 4, I2C_FREQ)) {
      break;
    }

    knob = ((reg00[3] << 24) & 0xff000000)|
           ((reg00[2] << 16) & 0x00ff0000)|
           ((reg00[1] <<  8) & 0x0000ff00)|
           ((reg00[0] <<  0) & 0x000000ff);

    // ハードのバグ対応
    knob /= 2;

    /*
     * ボタン状態の読み出し
     * ※ボタンの押下状態がレベルで出力され、開放時に1が、押下時に0が帰ってくる
     */
    if (!M5.Ex_I2C.readRegister(ENCODER_ADDR, 0x20, &reg20, 1, I2C_FREQ)) {
      break;
    }

    btn  = (reg20 != 1 && prev == 1);
    prev = reg20;

    /*
     * ノブの積算回転量のリセット
     */
    if (knob != 0) {
      if (!M5.Ex_I2C.writeRegister(ENCODER_ADDR, 0x40, &reg40, 1, I2C_FREQ)) {
        break;
      }
    }

    ret  = true;
  } while (false);

  if (ret) {
    *dst1 = knob;
    *dst2 = btn;
  }

  return ret;
}

/**
 * 秒経過音再生
 */
static void beep1()
{
  switch (soundMode) {
  case SOUND_MODE1:
    M5.Speaker.tone(1200, 10);
    break;

  case SOUND_MODE2:
  case SOUND_MODE3:
    // なにもしない
    break;
  }
}

/**
 * 10秒経過音再生
 */
static void beep2()
{
  switch (soundMode) {
  case SOUND_MODE1:
    M5.Speaker.tone(3200, 50);
    break;

  case SOUND_MODE2:
  case SOUND_MODE3:
    // なにもしない
    break;
  }
}

/**
 * タイムアウト音再生
 */
static void beep3()
{
  switch (soundMode) {
  case SOUND_MODE1:
  case SOUND_MODE2:
    M5.Speaker.tone(2000, 1500);
    break;

  case SOUND_MODE3:
    // なにもしない
    break;
  }
}

/**
 * カウントダウン開始・停止音再生
 */
static void beep4()
{
  switch (soundMode) {
  case SOUND_MODE1:
  case SOUND_MODE2:
    M5.Speaker.tone(1600, 100);
    break;

  case SOUND_MODE3:
    // なにもしない
    break;
  }
}

/**
 * カウント値変更操作音再生
 */
static void beep5()
{
  switch (soundMode) {
  case SOUND_MODE1:
  case SOUND_MODE2:
    M5.Speaker.tone(8000, 10);
    break;

  case SOUND_MODE3:
    // なにもしない
    break;
  }
}

/**
 * カウントダウンコールバック
 *
 * @param [in] handle  タイマーのハンドラ (未使用)
 *
 * @remarks
 *  本関数は、経過時間カウント用のカウンタのインクリメント及び、ブザーの鳴動を
 *  行う。本関数はタイマーにより1秒ごとに呼び出されることを前提としている。
 */
static void
count_down_timer(TimerHandle_t handle)
{
  /*
   * クリティカルセクションの開始
   */
  if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
    /*
     * 時刻カウンタのインクリメント
     */
    master.count--;

    /*
     * 毎秒ごとの経時音は1200Hzで行うが、10秒ごとに3200Hzのサイン波で鳴動させ
     * る。
     */
    if (master.count == 0) {
      xTimerStop(timer1, 0);
      beep3();
      state = STATE_TIMEDOUT;

    } else if (master.count % 10 == 0) {
      beep2();

    } else {
      beep1();
    }

    /*
     * データの更新をマーク
     */
    master.updated = true;

    /*
     * クリティカルセクションの終了
     */
    xSemaphoreGive(mutex);
  }
}

/**
 * バッテリーレベル取得コールバック
 *
 * @param [in] handle  タイマーのハンドラ (未使用)
 *
 * @remarks
 *  本関数は、バッテリーレベルの取得を行う。タイマーにより定期的に呼び出される
 *  ことを前提としている。
 */
static void
battery_timer(TimerHandle_t handle)
{
  int val;

  /*
   * クリティカルセクションの開始
   */
  if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
    /*
     * バッテリレベルの読み出し
     */
    val = M5.Power.getBatteryLevel();

    if (master.battery == 0 && val > 0) {
      xTimerChangePeriod(handle, pdMS_TO_TICKS(BATTERY_INTERVAL), 0);
    }

    /*
     * データの更新をマーク
     */
    if (master.battery != val) {
      master.battery = val;
      master.updated = true;
    }

    /*
     * クリティカルセクションの終了
     */
    xSemaphoreGive(mutex);
  }
}

/**
 * ボタン状態の評価
 *
 * @remarks
 *  クリティカルセクションが細切れになるのを防ぐため、関数全体をクリティカルセ
 *  クションとしている。また、本関数でクリティカルセクションを設けているので、
 *  表示用データの読み出しも本関数で行っている。
 */
static void
eval_input()
{
  int rot;
  bool btn;

  /*
   * ボタン状態の更新
   */
  M5.update();

  /*
   * ロータリーエンコーダの状態の読み出し
   */
  read_encoder_state(&rot, &btn);

  /*
   * クリティカルセクションの開始
   */
  if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
    // 本体ボタンの評価
    if (M5.BtnA.wasClicked()) {
      // ホームボタンがクリックされている場合は、動作状態の切り替え
      switch (state) {
      case STATE_READY:
        if (isChange) backup = master.count;
        beep4();
        xTimerStart(timer1, 0);
        state = STATE_COUNTDOWN;
        break;

      case STATE_COUNTDOWN:
        xTimerStop(timer1, 0);
        beep4();
        state = STATE_READY;
        break;

      case STATE_TIMEDOUT:
        master.count = backup;
        state = STATE_READY;
        break;
      }

      reset_encoder();
      master.updated = true;
      isChange = false;

    } else if (M5.BtnA.wasHold()) {
      // ホームボタンの長押しは、編集時の元の値への復帰
      if (state == STATE_READY) {
        beep5();
        master.count   = backup;
        master.updated = true;
        isChange = false;
      }

    } else if (M5.BtnB.wasClicked()) {
      // Bボタンクリックは、ブザーON/OFFのトグル切り替え
      switch (soundMode) {
      case SOUND_MODE1:
        soundMode = SOUND_MODE2;
        break;

      case SOUND_MODE2:
        soundMode = SOUND_MODE3;
        break;

      case SOUND_MODE3:
        soundMode = SOUND_MODE1;
        break;
      }

      beep5();
      master.updated = true;

    } else if (M5.BtnPWR.wasClicked()) {
      // 電源ボタンクリックは電源OFF
      M5.Power.powerOff();
    }

    if (state == STATE_READY) {
      // ロタリーエンコーダのノブの回転量の評価
      if (rot != 0) {
        beep5();

        switch (stepMode) {
        case STEP_MODE_MIN:
          master.count += (rot * 60);
          break;

        case STEP_MODE_10SEC:
          master.count += (rot * 10);
          break;

        case STEP_MODE_1SEC:
          master.count += rot;
          break;
        }

        if (master.count <= 0) {
          master.count += 3600;

        } else if (master.count > 3600) {
          master.count -= 3600;
        }

        master.updated = true;
        isChange = true;
      }

      // ロタリーエンコーダのボタンの状態の評価
      if (btn) {
        switch (stepMode) {
        case STEP_MODE_MIN:
          stepMode = STEP_MODE_10SEC;
          break;

        case STEP_MODE_10SEC:
          stepMode = STEP_MODE_1SEC;
          break;

        case STEP_MODE_1SEC:
          stepMode = STEP_MODE_MIN;
          break;
        }

        beep5();
        master.updated = true;
        isChange = true;
      }
    }

    /*
     * クリティカルセクションの終了
     */
    xSemaphoreGive(mutex);
  }
}

/**
 * 画面表示の更新
 */
void
update_display()
{
  data_set_t display;
  char str[40];
  int w;
  int h;
  int x;
  int y;
  int col;

  /*
   * クリティカルセクションの開始
   */
  if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
    // 表示用データへのコピー
    display = master;

    // 更新フラグのクリア
    master.updated = false;

    /*
     * クリティカルセクションの終了
     */
    xSemaphoreGive(mutex);
  }

  /*
   * データ更新がマークされている場合のみ描画処理を実施
   */
  if (display.updated) {
    /*
     * オフスクリーンウィンドウのクリア
     */
    canvas.fillScreen(TFT_BLACK);

    /*
     * 7セグ箇所の描画
     */
    canvas.setTextFont(7);
      
    // 影の描画
    w = canvas.textWidth("88:88");
    h = canvas.fontHeight();
    x = (M5.Display.width() - w) / 2;
    y = (M5.Display.height() - h) / 2;

    canvas.setCursor(x, y);
    canvas.setTextColor(canvas.color565(10, 10, 10));
    canvas.print("88:88");

    // 数値の描画
    sprintf(str, "%2d:%02d", display.count / 60, display.count % 60);
    x = (x + w) - canvas.textWidth(str);

    switch (state) {
    case STATE_COUNTDOWN:
      // カウントダウン時はグリーン
      canvas.setTextColor(TFT_GREEN);
      canvas.setCursor(x, y);
      canvas.print(str);
      break;

    case STATE_TIMEDOUT:
      // タイムアウト時の0:00は赤
      canvas.setTextColor(TFT_RED);
      canvas.setCursor(x, y);
      canvas.print(str);
      break;

    case STATE_READY:
      if (isChange) {
        // 変更中の場合は、変更対象の桁を青で、その他の桁はグリーンで表示
        sprintf(str, "%2d", display.count / 60);
        col = (stepMode == STEP_MODE_MIN)? TFT_SKYBLUE: TFT_GREEN;
        canvas.setTextColor(col);
        canvas.setCursor(x, y);
        canvas.print(str);
        x += canvas.textWidth(str);

        canvas.setTextColor(TFT_GREEN);
        canvas.setCursor(x, y);
        canvas.print(":");
        x += canvas.textWidth(":");

        sprintf(str, "%d", (display.count % 60) / 10);
        col = (stepMode == STEP_MODE_10SEC)? TFT_SKYBLUE: TFT_GREEN;
        canvas.setTextColor(col);
        canvas.setCursor(x, y);
        canvas.print(str);
        x += canvas.textWidth(str);

        sprintf(str, "%d", (display.count % 60) % 10);
        col = (stepMode == STEP_MODE_1SEC)? TFT_SKYBLUE: TFT_GREEN;
        canvas.setTextColor(col);
        canvas.setCursor(x, y);
        canvas.print(str);

      } else {
        // 変更が行われていない場合は、すべてグリーンで表示
        canvas.setTextColor(TFT_GREEN);
        canvas.setCursor(x, y);
        canvas.print(str);
      }

      break;
    }

    /*
     * バッテリーレベルの描画
     */
    if (display.battery > 0) {
      int bat;
      int gap = 3;
      int wd = (M5.Display.width() - ((gap * 9) + (5 * 2))) / 10;
      int ht = 7;
      int x = (M5.Display.width() - ((wd * 10) + (gap * 9))) / 2;
      int y = M5.Display.height() - (ht + 3);
      int col;

      for (bat = 90; bat >= 0; bat -= 10) {
        if (display.battery >= bat) {
          col = canvas.color565(192, 128, 0);
        } else {
          col = canvas.color565(10, 10, 10);
        }

        canvas.setColor(col);
        canvas.fillRoundRect(x, y, wd, ht, 2);

        x += (gap + wd);
      }
    }

    /*
     * サイレントモードインジケータの描画
     */
    x = M5.Display.width() - (16 + 7);
    y = 7;

    switch (soundMode) {
    case SOUND_MODE1:
      canvas.pushImage(x, y, 16, 16, note, 0x07e0);
      break;

    case SOUND_MODE2:
      canvas.pushImage(x, y, 16, 16, note2, 0x07e0);
      break;

    case SOUND_MODE3:
      canvas.fillCircle(x + 6, y + 7, 11, TFT_BLUE);
      canvas.pushImage(x, y, 16, 16, note, 0x07e0);
      canvas.fillArc(x + 6, y + 7, 11, 10, 0, 360, TFT_RED);
      canvas.drawLine(x - 1, y + 1, x + 14, y + 13, TFT_RED);
      canvas.drawLine(x - 1, y + 2, x + 14, y + 14, TFT_RED);
      canvas.drawLine(x - 1, y + 3, x + 14, y + 15, TFT_RED);
      canvas.drawLine(x - 0, y + 3, x + 13, y + 15, TFT_RED);
      break;
    }

    /*
     * オフスクリーンウィンドウの内容をLCDに反映
     */
    M5.Display.startWrite();
    canvas.pushSprite(0, 0);
    M5.Display.endWrite();
  }
}

/**
 * 設定処理
 */
void
setup()
{
  /*
   * ボード本体の初期化
   */
  M5.begin();

  /*
   * ディスプレイの初期化
   */
  M5.Display.setRotation(1);

  /*
   * I2Cの初期化
   */
  M5.Ex_I2C.begin(I2C_NUM_0, SDA, SCL);

  /*
   * ボードごとの設定
   */
  switch (M5.getBoard()) {
  case m5::board_t::board_M5StickC:
    M5.Display.setBrightness(BRIGHTNESS);
    break;

  case m5::board_t::board_M5StickCPlus:
    M5.Display.setBrightness(BRIGHTNESS);
    M5.Speaker.setVolume(255);
    break;

  case m5::board_t::board_M5StickCPlus2:
    M5.Display.setBrightness(BRIGHTNESS / 3);
    M5.Speaker.setVolume((BUZZER_VOLUME * 3) / 5);
    break;
  }

  /*
   * オフスクリーンウィンドウの初期化
   */
  canvas.setColorDepth(16);
  canvas.createSprite(M5.Display.width(), M5.Display.height());

  /*
   * 排他制御用ミューテックスの生成
   */
  mutex = xSemaphoreCreateMutex();

  /*
   * 変数の初期化
   */
  master    = {180, 0, true};
  backup    = master.count;
  state     = STATE_READY;
  isChange  = false;
  stepMode  = STEP_MODE_10SEC;
  soundMode = SOUND_MODE1;

  reset_encoder();

  /*
   * カウントアップタイマーの設定
   */
  timer1 = xTimerCreate("count up timer",
                       pdMS_TO_TICKS(TIMER_INTERVAL),
                       pdTRUE,
                       NULL,
                       count_down_timer);

  /*
   * バッテリレベル取得タイマーの設定と起動
   */
  timer2 = xTimerCreate("battery timer",
                       pdMS_TO_TICKS(1000),
                       pdTRUE,
                       NULL,
                       battery_timer);

  xTimerStart(timer2, 0);
}

/**
 * UI制御ルーパー
 */
void
loop()
{
  unsigned long t0 = millis();

  /*
   * ユーザ入力の評価
   */
  eval_input();

  /*
   * 画面の更新
   */
  update_display();

  /*
   * 次の処理までのウェイト
   */
  delay(LOOP_INTERVAL - (millis() - t0));
}
