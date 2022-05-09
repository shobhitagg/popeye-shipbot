#include "UTModule.hpp"


double UTModule::getThetaError(){

	double distanceFR = ultrasonicFR ();
	double distanceFL = ultrasonicFL ();

	double theta = atan ((distanceFR - distanceFL)/utLx);
 
	return theta;
}

double UTModule::ultrasonicR(){
  
  long distance[5];
  long duration;

  for(int i=0; i<5; i++){
    digitalWrite(rTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(rTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(rTrig, LOW);
    duration = pulseIn(rEcho, HIGH);
    distance[i] = (duration / 2) / 29.1; //Stores preliminary reading to compare
  }

  return(median(distance, 5));
}

double UTModule::ultrasonicL(){
  
  long distance[5];
  long duration;

  for(int i=0; i<5; i++){
    digitalWrite(lTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(lTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(lTrig, LOW);
    duration = pulseIn(lEcho, HIGH);
    distance[i] = (duration / 2) / 29.1; //Stores preliminary reading to compare
  }

  return(median(distance, 5));
}


double UTModule::ultrasonicFR(){
  
  long distance[5];
  long duration;

  for(int i=0; i<5; i++){
    digitalWrite(frTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(frTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(frTrig, LOW);
    duration = pulseIn(frEcho, HIGH);
    distance[i] = (duration / 2) / 29.1; //Stores preliminary reading to compare
  }

  return(median(distance, 5));
}


double UTModule::ultrasonicFL(){
  
  long distance[5];
  long duration;

  for(int i=0; i<5; i++){
    digitalWrite(flTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(flTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(flTrig, LOW);
    duration = pulseIn(flEcho, HIGH);
    distance[i] = (duration / 2) / 29.1; //Stores preliminary reading to compare
  }

  return(median(distance, 5));
}

double UTModule::ultrasonicF(){
	return((ultrasonicFL()+ultrasonicFR())/2);
}

double UTModule::median(long *arr, int n){

    int i, key, j;
    for (i = 1; i < n; i++)
    {
        key = arr[i];
        j = i - 1;
 
        /* Move elements of arr[0..i-1], that are
        greater than key, to one position ahead
        of their current position */
        while (j >= 0 && arr[j] > key)
        {
            arr[j + 1] = arr[j];
            j = j - 1;
        }
        arr[j + 1] = key;
    }

  if(n%2 == 1)
    return(arr[n/2]);
  else 
    return(arr[n/2-1]);
  
}


/*
long UTModule::ultrasonicL(int sample=5){
  float duration;
  float distance = 0;
  int ctr = 0;
  delay(50);
  while (ctr<sample)
  {
    digitalWrite (lTrig, HIGH);
    delayMicroseconds (10);
    digitalWrite (lTrig, LOW);
    delayMicroseconds (2);
    
    duration = pulseIn (lEcho, HIGH);
    distance += (duration*0.034)/2;
    ctr++;
  }
  return distance/sample;
}
*/
