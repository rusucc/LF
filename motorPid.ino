IntervalTimer crono;

const int dt = 50; // ms pentru update motoare etc
volatile int count0 = 0;
volatile int count1 = 0;

void encA() {
  count0++;
}
void encB(){
  count1++;
}
class PID {
  protected:
    double KP, KI, KD;
    int integral = 0, ev = 0;
  public:
    PID(double KP=0, double KI=0, double KD=0) {
      this->KP = KP;
      this->KI = KI;
      this->KD = KD;
    }
    void set(double KP, double KI, double KD){
      this->KP = KP;
      this->KI = KI;
      this->KD = KD;
    }
    double calculateOutput(int target, int current) {
      int e = current - target;
      integral += e;
      int out = KP * e + KI * integral + KD * (e - ev);
      ev = e;
      return out;
    }
};
class Motor: public PID {
  private:
    volatile int count;
    double speed, targetSpeed,distance;
    struct pins {
      int IN1, IN2, enc;
    } p;
    double r, ppr,reductor; // r in cm
    inline void calculateSpeed(){
      distance = count * 2 * PI * r / ppr / reductor;
      speed = distance * 10/ dt; // metri pe secunda
    }
  public:
    Motor(pins p,double KPM, double KIM, double KDM) : PID() {
      this->set(KPM,KIM,KDM);
      this->p.IN1 = p.IN1;
      this->p.IN2 = p.IN2;
      this->p.enc = p.enc;
    }
    void setTargetSpeed(double v){
      targetSpeed = v;
    }
    void inline update(volatile int count){
      this->count = count; 
    }
    pins getPins(){
      return p;
    }
};
Motor M1=Motor({2,3,4},0,0,0);
Motor M2=Motor({5,6,7},0,0,0);
PID Senzori;
void refresh(){
  M1.update(count0);
  count0=0;
  M2.update(count1);
  count1=0;
  Serial.println("In interrupt!!!");
}
void setup() {
  crono.begin(refresh,dt*1000);//el vrea microsecunde, 1000 pt milisecunde
  attachInterrupt(digitalPinToInterrupt(M1.getPins().enc), encA, RISING);
  attachInterrupt(digitalPinToInterrupt(M2.getPins().enc), encB,RISING);
  Serial.begin(9600);
  Serial1.begin(9600); //bluetooth
  Serial1.println("Hello");
}
void loop() {
  bool run = false;
  if (Serial1.available()) {
    if(Serial1.read()=='1') run = true;
    else run = false;
  }
  delay(10);
}
