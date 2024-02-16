#define BT Serial1
#define number 6 //nr senzori
IntervalTimer crono;
IntervalTimer telemetrie;

const int dt = 50; // ms pentru update motoare 
const int dts = 10; // ms pentru senzori
volatile int count0 = 0;
volatile int count1 = 0;
long T; //timpul in ms aici 

void encA() {
  count0++;
}//functie apelata pe interrupt pentru motorul A
void encB() {
  count1++;
}//functie apelata pe interupt pentru motorul B
class PID {
  protected:
    double KP, KI, KD, integral = 0, e_old = 0;
    //e_old-eroare veche
  public:
    PID(double KP = 0, double KI = 0, double KD = 0) {
      this->KP = KP;
      this->KI = KI;
      this->KD = KD;
    }
    void set(double KP, double KI, double KD) {
      this->KP = KP;
      this->KI = KI;
      this->KD = KD;
    }
    double calculateOutput(double target, double current) {
      double e = current-target;
      integral += (KI * e);
      integral=(integral>4000)? 4000:integral;
      integral=(integral<-4000)? -4000:integral;
      double out = KP * e + integral + KD * (e - e_old);
      e_old = e;
      return out;
    }
};
class Motor: public PID {
  public:
    volatile int count;
    double speed, targetSpeed, distance;
    struct pins {
      int IN1, IN2, enc;
    } p;
    struct roata{
      float radius, ppr, reductor; // r in cm
    } r;
    float PWM = 0;
    int out = 0;
    inline void calculateSpeed() {
      distance = count * 2 * PI * r.radius / r.ppr / r.reductor;
      speed = distance * 1000 / dt; // cm pe secunda
    }
    Motor(pins p, double KPM, double KIM, double KDM, roata r) : PID() {
      this->KP = KPM;
      this->KI = KIM;
      this->KD = KDM;
      this->p.IN1 = p.IN1;
      this->p.IN2 = p.IN2;
      this->p.enc = p.enc;
      this->r.radius= r.radius;
      this->r.ppr = r.ppr;
      this->r.reductor = r.reductor;
    }
    void setTargetSpeed(double v) {
      targetSpeed = v;
    }
    void inline update(volatile int count) {
      this->count = count;
    }
    void run() {
      calculateSpeed();
      PWM=(PWM<0)? 0:PWM;
      PWM=(PWM>255)? 255:PWM;
      analogWrite(p.IN1, PWM);
      analogWrite(p.IN2, 0);
    } 
    void setPWM(int val){
      PWM=val;
    }
    pins getPins(){
      return p;
    }
    String printV(){
      return "V: " + String(speed) + " Vt: " + String(targetSpeed) + " Output PID: " + String(out)+" PWM: " + String(PWM);
    }
    String printPID(){
      return "P: " + String(KP) + " I: " + String(KI) + " D: " + String(KD) + " E: "+String(speed-targetSpeed)+" O: "+String(out)+" I:" + integral;
    }
    String printAll(){
      return "V: " + String(speed) + " Vt: " + String(targetSpeed) + " Output PID: " + String(out)+" PWM: " + String(PWM)+ " E: "+String(speed-targetSpeed)+" I:" + integral;
    }
    String printCSV(){//v,vt,target,out,pwm,e,i
      return ","+String(speed)+"," + String(targetSpeed) + "," + String(out)+"," + String(PWM)+ ","+String(speed-targetSpeed)+"," + integral;
    }
};
class Sensors: public PID{
  public:
    bool line;
    struct calibrated_values{
      int max_value;
      int min_value;
    } calib[number];
    int values[number];
    int pins[number];
    const int threshold = 700;
    int position;
    void calculatePosition(){
      int sum = 0;
      int w_avg = 0;
      line = false;
      for(int i = 0; i < number; i++){
        while(T-millis()<dts);
        values[i] = map(analogRead(pins[i]),calib[i].min_value,calib[i].max_value,0,1000);
        T=millis();
        if(values[i]>threshold) line = true;
        sum+=values[i];
        w_avg += (i*1000)*values[i];
      }
      if(sum>0) position = w_avg/sum;
      else return;
    }
    void calibrate(int cycles){
      for(int c = 0; c < cycles; c++){
        for(int i = 0; i < number; i++){
          while(millis()-T<dts);
          T = millis();
          int temp_value = analogRead(pins[i]);
          calib[i].min_value=min(calib[i].min_value,temp_value);
          calib[i].max_value=max(calib[i].max_value,temp_value);
        }
      }
    }
    Sensors(double KPS, double KIS, double KDS, int pins[]) : PID() {
      this->KP = KPS;
      this->KI = KIS;
      this->KD = KDS;
      memcpy(this->pins,pins,number*sizeof(int));//daca nu merge, asta e
    }
};
int pins_sensors[number]={0, 0, 0, 0, 0 , 0};
Motor M1 = Motor({2, 3, 4}    , 0,0,0, {1, 3, 30});
Motor M2 = Motor({33, 34, 35} , 0.25, 0.01, 0.2, {1, 3, 30}); //Motor M({IN1,IN2,enc},KP,KI,KD,{radius[cm],ppr,reductor});
Sensors QRE(0, 0, 0, pins_sensors);
void refresh() {
  M1.update(count0);
  M1.out = M1.calculateOutput(M1.targetSpeed, M1.speed);
  M1.PWM-=M1.out;
  count0 = 0;
  M2.update(count1);
  M2.out = M2.calculateOutput(M2.targetSpeed, M2.speed);
  M2.PWM-=M2.out;
  count1 = 0;
}
void setup() {
  Serial.begin(9600);
  BT.begin(9600); //bluetooth
  BT.println("Hello");
  crono.begin(refresh, dt * 1000); //el vrea microsecunde, 1000 pt milisecunde
  telemetrie.begin([](){BT.println(M1.printCSV());},300*1000);//telemetrie la fiecare secunda
  attachInterrupt(digitalPinToInterrupt(M1.getPins().enc), encA, RISING);
  attachInterrupt(digitalPinToInterrupt(M2.getPins().enc), encB, RISING);
}
void loop() {
  if (BT.available()) {
    String s = BT.readString(); //din aplicatie vine cu \n
    s.toLowerCase();
    s.trim();
    String mesaj[5];
    int mesaje = 0;
    while (s.length() > 0) {
      int index = s.indexOf(' ');
      if (index == -1) {
        mesaj[mesaje++] = s;
        break;
      }
      else mesaj[mesaje++] = s.substring(0, index), s = s.substring(index + 1);
    }
    BT.println(mesaj[0] + " " + mesaj[1]);
    if (mesaj[0] == "m1") {
      if (mesaj[1] == "printv") BT.println(M1.printV());
      else if (mesaj[1] == "printpid") BT.println(M1.printV()),BT.println(M1.printPID());
      else if (mesaj[1] == "pid"){
        M1.set(mesaj[2].toFloat(), mesaj[3].toFloat(), mesaj[4].toFloat());
      }
      else if(mesaj[1] == "pwm") M1.setPWM(mesaj[2].toFloat());
      else M1.setTargetSpeed(mesaj[1].toFloat()), BT.println("Setat: " + mesaj[0] + " cu valoarea: " + mesaj[1]);;
    }
  }
  M1.run();
  Serial.println(M1.printV());
}
