Un pic si de documentatie ca sa nu ma bat singur:

Motor clasa mostenita din PID

Pentru a initializa, folosim:
```
PID(double KP, double KI, double KD)
```
iar pentru output, folosim:
```
double calculateOutput(double target, double current)
```

Pentru motor, avem
```
Motor M({IN1,IN2,enc},KP,KI,KD,r,ppr,reductor);
setTargetSpeed(double v) // m/s
```

Functia refresh este apelata folosind timerInterrupt, la cateva ms
In aceasta functie, setam pozitia encoderelor in obiect si resetam counterul
```
void refresh(){
  M1.update(count0);
  count0=0;
  M2.update(count1);
  count1=0;
  Serial.println("In interrupt!!!");
}
```
