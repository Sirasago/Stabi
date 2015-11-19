//A,Bで二軸分制御する
#define KP_A 14 //比例要素
#define KP_B 14
#define KD_A 2 //微分要素。ループ処理時間と効果が関係する
#define KD_B 2
#define GYRO_A 0
#define GYRO_B 1
#define FIN_A 3 //正転信号
#define FIN_B 10
#define RIN_A 11 //逆転信号
#define RIN_B 9
#define P_A 0.9999//ループ時間Δtについて時定数τ=-Δt/iognaturalPであり、遮断周波数fc=1/2πτである。ΔtとPでローパスフィルタを構成する
#define P_B 0.9999
#define Q_A 0.50
#define Q_B 0.50
char pulse = 0; //ループ時間測定用1
float offset_a  = 655; //オフセット3.1Vにドリフトが重畳するのでこれを初期値としてジャイロのアナログ電圧の重み付き時間平均で作成
float offset_b  = 655;
float myn_a = 0;
float myn_b = 0;
float myn_1a = 0;
float myn_1b = 0;
float en_a = 0;
float en_b = 0;
float en_1a = 0;
float en_1b = 0;
float en_2a = 0;
float en_2b = 0;
float coil_a = 0;
float coil_b = 0;
inline void driftcorrect_a(void) {
  offset_a = analogRead(GYRO_A) * (1 - P_A) + offset_a * P_A;
}
inline void driftcorrect_b(void) {
  offset_b = analogRead(GYRO_B) * (1 - P_B) + offset_b * P_B;
}
inline void smooth_a(void) {
  en_a = (analogRead(GYRO_A) - offset_a) * (1 - Q_A) + en_a * Q_A;
}
inline void smooth_b(void) {
  en_b = (analogRead(GYRO_B) - offset_b) * (1 - Q_B) + en_b * Q_B;
}
inline void coilmove_a(void)
{
  coil_a = constrain(myn_a, -255, 255);
  if (0 < coil_a ) {
    analogWrite(FIN_A, coil_a);
    digitalWrite(RIN_A, 0);
    return;
  }
  if (coil_a <= 0) {
    digitalWrite(FIN_A, 0);
    analogWrite(RIN_A, -coil_a);
    return;
  }
}
inline void coilmove_b(void)
{
  coil_b = constrain(myn_b, -255, 255);
  if (0 < coil_b ) {
    analogWrite(FIN_B, coil_b);
    digitalWrite(RIN_B, 0);
    return;
  }
  if (coil_b <= 0) {
    digitalWrite(FIN_B, 0);
    analogWrite(RIN_B, -coil_b);
    return;
  }
}
void setup()
{
  TCCR1B &= B11111000; //PWM周波数高速化のレジスタ操作。9,10番ピンが31.4kHzになる。
  TCCR1B |= B00000001;
  TCCR2B &= B11111000; //PWM周波数高速化のレジスタ操作。3,11番ピンが31.4kHzになる。
  TCCR2B |= B00000001;
  ADCSRA = ADCSRA & 0xf8;//analogRead高速化。8倍速で19.048usになる。
  ADCSRA = ADCSRA | 0x04;
  pinMode(FIN_A, OUTPUT);
  pinMode(FIN_B, OUTPUT);
  pinMode(RIN_A, OUTPUT);
  pinMode(RIN_B, OUTPUT);
  pinMode(13, OUTPUT); //ループ時間測定用2
  // Serial.begin(9600);//アナログジャイロセンサのゼロ点校正用1PWM高速化と同時使用できない
}
void loop()
{
  driftcorrect_a();
  smooth_a();
  myn_a = KP_A * (en_a - en_1a) + KD_A * (en_a - 2 * en_1a + en_2a) + myn_1a;
  coilmove_a();
  myn_1a = myn_a;
  en_2a = en_1a;
  en_1a = en_a;

  driftcorrect_b();
  smooth_b();
  myn_b = KP_B * (en_b - en_1b) + KD_B * (en_b - 2 * en_1b + en_2b) + myn_1b;
  coilmove_b();
  myn_1b = myn_b;
  en_2b = en_1b;
  en_1b = en_b;
  digitalWrite(13, pulse); //ループ時間測定用3
  pulse = 1 - pulse; //ループ時間測定用4
  // Serial.print(offset);//アナログジャイロセンサのゼロ点校正用2
  // Serial.println(" ");//アナログジャイロセンサのゼロ点校正用3
}
