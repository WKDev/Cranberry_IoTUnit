#define RELAY_PIN D4

int turnOnMin = 0;
int turnOffMin = 0;
int totalElasped = 0;
int relayState = 1;

unsigned int startTime = millis();




void setup(){
	unsigned int startTime = millis();
}

void loop()
{ 
  elaspedSec = (millis() - startTime) / 1000;
  elaspedMin = (elaspedSec / 60); 
 
  if(relayState){
	  if(turnOnMin<20){
		  turnOnMin++;
	  }
	  else{
		  turnOnMin = 0;
		  relayState = 1;
	  }
  }
  else if(!relayState){
	  if(turnOffMin<40){
		  turnOffMin++;
	  }
	  else{
		  turnOffMin = 0;
		  relayState = 0;
	  }
		  
  delay(1000);
  digitalWrite(RELAY_PIN, relayState);
}
