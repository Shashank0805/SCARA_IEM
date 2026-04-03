float d;
int a,i=0,c,b;
unsigned long t;
void setup() {
  pinMode(A0,INPUT);//pot output
  pinMode(A1,INPUT);//initiator(initially connect with Vin; when connected to vcc, reading starts)
  pinMode(A2,INPUT);//volatge divider
  Serial.begin(9600);
}
void loop() {
  if(i<1000 && analogRead(A1)==0){
    t = millis();
    a=analogRead(A0);
    b=analogRead(A2);
    Serial.print(t);//timestamps in ms
    Serial.print(" ");
    Serial.print(a*5.0/1023);//position
    Serial.print(" ");
    Serial.print(b*5.0*2.0/1023);//voltage
    Serial.print(";");
    i+=1;
    delay(2);
  }
}