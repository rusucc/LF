Un pic si de documentatie ca sa nu ma bat singur:

Motor clasa mostenita din PID

#Clasa PID:
## Constructor:
```
PID(double KP, double KI, double KD)
```
## Output:
```
double calculateOutput(double target, double current)
```
## Set
```
void set(double KP, double KI, double KD)
```

# Motor
## Constructor
```
Motor(pins p, double KPM, double KIM, double KDM, roata r)
```
## Target
```
setTargetSpeed(double v)
```
## Functie de update:
```
void inline update(volatile int count)
```
Metoda apelata indirect, cu ajutorul unei alte functii de refresh, care la randul ei este apelata cu timer interrupt
## Run
```
void run()
```
Functie apelata in loop, actualizeaza variabila *speed* din obiect si trimite semnal catre motor
**PID NU este rulat aici, fiind prea rapid** 

## SetPWM 
**Utilizat doar din interfata bluetooth**
```
void setPWM(int val)
```
## Telemetrie:
### printV: V, VT, out, PWM
```
String printV() 
```
### printPID: KP,KI,KD,E,O,I
### printAll: V,VT,out,PWM,E,I
### printCSV: v,vt,target,out,pwm,e,i
```
String printCSV()
```

# Programul principal 
## Motoare
```
Motor M({IN1,IN2,enc},KP,KI,KD,{raza[cm],ppr,reductor});
```
## Refresh
Functia refresh este apelata folosind timerInterrupt, la cateva ms
In aceasta functie, setam pozitia encoderelor in obiect si resetam counterul
```
volatile int count
```
```
void refresh() {
  M.update(count);
  M.out = M.calculateOutput(M.targetSpeed, M.speed);
  M.PWM-=M.out;
  count = 0;
}
```
## Setup:
dt in microsecunde
SerialX - documentatie Teensy
```
#define BT SerialX
BT.begin(baud rate)
timer.begin(refresh, dt)
```
