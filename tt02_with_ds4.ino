#include <PS4BT.h>
#include <usbhub.h>
#include <SPI.h>
#include <Servo.h>

USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd); 

Servo steeringServo; // D5: ステアリング
Servo escServo;      // D6: ESC

/*
 * ============================================================================
 * 使用ピン一覧 (Arduino Uno / Nano 等 ATmega328Pボード基準)
 * ============================================================================
 * 
 * 【1. シリアル通信 (Serial)】
 *  - D0 (RX)  : シリアル受信 (USBシリアル通信で使用)
 *  - D1 (TX)  : シリアル送信 (USBシリアル通信で使用)
 * 
 * 【2. サーボ・ESC制御 (Servo.h ＆ ユーザー定義)】
 *  - D5       : ステアリングサーボ信号出力 (SERVO_PIN)
 *  - D6       : ESC (スピードコントローラー) 信号出力 (ESC_PIN)
 *  ※ 注意: Servoライブラリ使用時は内部で「Timer1」を占有するため、
 *          D9, D10ピンのPWM出力(analogWrite)は使用できなくなります。
 *          (デジタル入出力やUSB Host Shieldの制御ピンとしては機能します)
 * 
 * 【3. USB Host Shield 2.0 (PS4BT.h / usbhub.h / SPI.h)】
 *  - D9       : INT (MAX3421E 割り込み信号)
 *  - D10      : SS / CS (MAX3421E SPIチップセレクト)
 *  - D11      : MOSI (SPI データ送信) ※ICSPヘッダ経由で接続される場合も内部的に占有
 *  - D12      : MISO (SPI データ受信) ※同上
 *  - D13      : SCK  (SPI クロック)   ※同上
 *  ※ 補足: シールド基板上のジャンパ設定によっては D7 または D8 が 
 *          GPX / RST ピンとして物理的に結線されている場合があります。
 * 
 * 【空きピン (追加のセンサーやLED等で利用可能)】
 *  - デジタル : D2, D3, D4, D7, D8
 *  - アナログ : A0, A1, A2, A3, A4, A5 (デジタル入出力としても使用可能)
 * ============================================================================
 */

const int SERVO_PIN = 5; // ステアリング
const int ESC_PIN   = 6; // ESC

const int STEERING_CENTER_US = 1500;
const int STEERING_RANGE_US  = 300; // 舵角範囲 (1200us ～ 1800us)

const int ESC_STOP_US  = 1500;
const int ESC_RANGE_US = 400;  // 出力範囲

const int DEADZONE = 12;

// シリアル表示タイマー (100ms周期)
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(115200);
  
  if (Usb.Init() == -1) {
    Serial.println(F("USB Host Shield 初期化失敗"));
    while (1);
  }
  Serial.println(F("USB Host Shield 初期化成功"));

  steeringServo.attach(SERVO_PIN);
  escServo.attach(ESC_PIN);

  steeringServo.writeMicroseconds(STEERING_CENTER_US);
  escServo.writeMicroseconds(ESC_STOP_US);
  
  delay(1000);
}

void loop() {
  Usb.Task();

  if (PS4.connected()) {
    // ----------------------------------------------------
    // 1. D5: ステアリング制御 (左スティック X軸)
    // ----------------------------------------------------
    uint8_t rawX = PS4.getAnalogHat(LeftHatX);
    int steeringUs = STEERING_CENTER_US;

    if (rawX < (127 - DEADZONE) || rawX > (127 + DEADZONE)) {
      steeringUs = map(rawX, 0, 255, STEERING_CENTER_US - STEERING_RANGE_US, STEERING_CENTER_US + STEERING_RANGE_US);
    }
    steeringServo.writeMicroseconds(steeringUs);

    // ----------------------------------------------------
    // 2. D6: アクセル・ブレーキ制御（反転修正済み）
    // ----------------------------------------------------
    uint8_t r2Val = PS4.getAnalogButton(R2);
    uint8_t l2Val = PS4.getAnalogButton(L2);
    int escUs = ESC_STOP_US;

    if (r2Val > 15) {
      // R2で前進（符号を反転: マイナス方向へ）
      escUs = map(r2Val, 0, 255, ESC_STOP_US, ESC_STOP_US - ESC_RANGE_US);
    } else if (l2Val > 15) {
      // L2で後進/ブレーキ（符号を反転: プラス方向へ）
      escUs = map(l2Val, 0, 255, ESC_STOP_US, ESC_STOP_US + ESC_RANGE_US);
    }
    escServo.writeMicroseconds(escUs);

    // ----------------------------------------------------
    // 3. デバッグ用シリアルプリント (100ms毎)
    // ----------------------------------------------------
    if (millis() - lastPrintTime > 100) {
      lastPrintTime = millis();
      Serial.print(F("[D5 Steer] RawX: "));
      Serial.print(rawX);
      Serial.print(F(" -> "));
      Serial.print(steeringUs);
      Serial.print(F("us | [D6 ESC] R2: "));
      Serial.print(r2Val);
      Serial.print(F(" L2: "));
      Serial.print(l2Val);
      Serial.print(F(" -> "));
      Serial.print(escUs);
      Serial.println(F("us"));
    }

    if (PS4.getButtonClick(PS)) {
      PS4.disconnect();
    }
  } else {
    // 切断時の安全保護
    steeringServo.writeMicroseconds(STEERING_CENTER_US);
    escServo.writeMicroseconds(ESC_STOP_US);
  }
}