#define BT Serial1
IntervalTimer crono;
IntervalTimer telemetrie;

const int dt = 50; // ms pentru update motoare etc
volatile int count0 = 0;
volatile int count1 = 0;

void encA() {
  count0++;
}
void encB() {
  count1++;
}
class PID {
  protected:
    double KP, KI, KD, integral = 0, ev = 0;
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
      double e = target-current;
      integral += e;
      double out = KP * e + KI * integral + KD * (e - ev);
      ev = e;
      return out;
    }
};
class Motor: public PID {
  private:
    float PWM = 0;
    int out = 0;
    volatile int count;
    double speed, targetSpeed, distance;
    struct pins {
      int IN1, IN2, enc;
    } p;
    double r, ppr, reductor; // r in cm
    inline void calculateSpeed() {
      distance = count * 2 * PI * r / ppr / reductor;
      speed = distance * 1000 / dt; // cm pe secunda
    }
  public:
    Motor(pins p, double KPM, double KIM, double KDM, double r, double ppr, double reductor) : PID() {
      this->KP = KPM;
      this->KI = KIM;
      this->KD = KDM;
      this->p.IN1 = p.IN1;
      this->p.IN2 = p.IN2;
      this->p.enc = p.enc;
      this->r = r;
      this->ppr = ppr;
      this->reductor = reductor;
    }
    void setTargetSpeed(double v) {
      targetSpeed = v;
    }
    void inline update(volatile int count) {
      this->count = count;
    }
    void run() {
      calculateSpeed();
      out = calculateOutput(targetSpeed, speed);
      PWM+=out;
      PWM=(PWM<0)? 0:PWM;
      PWM=(PWM>255)? 255:PWM;
      analogWrite(p.IN1, PWM);
      analogWrite(p.IN2, 0);
    }
    pins getPins() {
      return p;
    }
    String printV() {
      return "V: " + String(speed) + " Vt: " + String(targetSpeed) + " Output PID: " + String(out)+" PWM: " + String(PWM);
    }
    String printPID(){
      return "P: " + String(KP) + " I: " + String(KI) + " D: " + String(KD) + " E: "+String(speed-targetSpeed)+" O: "+String(calculateOutput(targetSpeed,speed));
    }
};
Motor M1 = Motor({2, 3, 4}    , 10, 0, 0, 1, 3, 30);
Motor M2 = Motor({33, 34, 35} , 10, 0, 0, 1, 3, 30); //Motor M({IN1,IN2,enc},KP,KI,KD,r,ppr,reductor);
PID Senzori;
void refresh() {
  M1.update(count0);
  count0 = 0;
  M2.update(count1);
  count1 = 0;
}
void setup() {
  Serial.begin(9600);
  BT.begin(9600); //bluetooth
  BT.println("Hello");
  crono.begin(refresh, dt * 1000); //el vrea microsecunde, 1000 pt milisecunde
  telemetrie.begin([](){BT.println(M1.printV());},500*1000);//telemetrie la fiecare secunda
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
      else M1.setTargetSpeed(mesaj[1].toFloat()), BT.println("Setat: " + mesaj[0] + " cu valoarea: " + mesaj[1]);;
    }
  }
  M1.run();
  Serial.println(M1.printV());
}
