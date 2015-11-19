//A,Bで二軸分制御する
#define KP_A 44 //比例要素
#define KP_B 44 
#define KI_A 0.0 //積分要素。
#define KI_B 0.0 
#define KD_A 2 //微分要素。ループ処理時間と効果が関係する
#define KD_B 2 
#define GYRO_A 0
#define GYRO_B 1
#define FIN_A 3 //正転信号
#define FIN_B 9 
#define RIN_A 11 //逆転信号
#define RIN_B 10 
#define P_A 0.9999//ループ時間Δtについて時定数τ=-Δt/iognaturalPであり、遮断周波数fc=1/2πτである。ΔtとPでローパスフィルタを構成する
#define P_B 0.9999
#define Q_A 0.93
#define Q_B 0.93
//int pulse = 0; //ループ時間測定用1
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
float driftcorrect_a(float s) {
  float d = analogRead(GYRO_A) * (1 - P_A) + s * P_A;
  return d;
}
float driftcorrect_b(float s) {
  float d = analogRead(GYRO_B) * (1 - P_B) + s * P_B;
  return d;
}
float smooth_a(float t) {
  float sm = (analogRead(GYRO_A) - offset_a) * (1 - Q_A) + t * Q_A;
  return sm;
}
float smooth_b(float t) {
  float sm = (analogRead(GYRO_B) - offset_b) * (1 - Q_B) + t * Q_B;
  return sm;
}
void coilmove_a(float a)
{
  float b = constrain(a, -255, 255);
  if (0 < b ) {
    analogWrite(FIN_A, b);
    analogWrite(RIN_A, 0);
    return;
  }
  if (b <= 0) {
    analogWrite(FIN_A, 0);
    analogWrite(RIN_A, -b);
    return;
  }
}
void coilmove_b(float a)
{
  float b = constrain(a, -255, 255);
  if (0 < b ) {
    analogWrite(FIN_B, b);
    analogWrite(RIN_B, 0);
    return;
  }
  if (b <= 0) {
    analogWrite(FIN_B, 0);
    analogWrite(RIN_B, -b);
    return;
  }
}
void setup()
{
  TCCR1B &= B11111000; //PWM周波数高速化のレジスタ操作。9,10番ピンが31.4kHzになる。
  TCCR1B |= B00000001;
  TCCR2B &= B11111000; //PWM周波数高速化のレジスタ操作。3,11番ピンが31.4kHzになる。
  TCCR2B |= B00000001;
  pinMode(FIN_A, OUTPUT);
  pinMode(FIN_B, OUTPUT);  
  pinMode(RIN_A, OUTPUT);
  pinMode(RIN_B, OUTPUT);
  //  pinMode(13,OUTPUT);//ループ時間測定用2
  // Serial.begin(9600);//アナログジャイロセンサのゼロ点校正用1PWM高速化と同時使用できない
}
void loop()
{
  offset_a = driftcorrect_a(offset_a);
  en_a = smooth_a(en_a);
  myn_a = KP_A * (en_a - en_1a) + KI_A * en_a + KD_A * (en_a - 2 * en_1a + en_2a) + myn_1a;
  coilmove_a(myn_a);
  myn_1a = myn_a;
  en_2a = en_1a;
  en_1a = en_a;
  
  offset_b = driftcorrect_b(offset_b);
  en_b = smooth_b(en_b);
  myn_b = KP_B * (en_b - en_1b) + KI_B * en_b + KD_B * (en_b - 2 * en_1b + en_2b) + myn_1b;
  coilmove_b(myn_b);
  myn_1b = myn_b;
  en_2b = en_1b;
  en_1b = en_b;
  // digitalWrite(13,pulse);//ループ時間測定用3
  // pulse = 1-pulse;//ループ時間測定用4
  // Serial.print(offset);//アナログジャイロセンサのゼロ点校正用2
  // Serial.println(" ");//アナログジャイロセンサのゼロ点校正用3
}
