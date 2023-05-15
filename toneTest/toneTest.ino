void setup() {
  // put your setup code here, to run once:
  pinMode(13,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if(analogRead(A0)>20){
    tone(13,analogRead(A0));
  }else{
    noTone(13);
  }
}
