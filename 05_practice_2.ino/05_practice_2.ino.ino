#define led  7
unsigned int light,cnt;
void setup() {
  pinMode(led,OUTPUT);
  Serial.begin(9600);
  light = 0;
  digitalWrite(led,light);
  delay(1000);
}

void loop() {
    
    light++;
    if(light>1){
      light = 0;
    }
    if(light==0){
      cnt++;
    }
    if(cnt==5){
      while(1){
      light =1;
      }
    }
    digitalWrite(led,light);
    Serial.println(light);
    
    delay(100);
  
}
