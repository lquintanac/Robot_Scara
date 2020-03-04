#include <PID_v1.h>
//////////////////////////////////////PINOUT DEL BRAZO///////////////////////////////////////


const byte    PWMA = 5; // MOTOR 1
const byte    PWMB = 6; // MOTOR 1
const byte    PWMC = 7; // MOTOR 2
const byte    PWMD = 8; // MOTOR 2
const byte    PWME = 9; // MOTOR 3
const byte    PWMF = 10;// MOTOR 3

const byte    encA = 2; //ENCODER 1
const byte    encB = 3; //ENCODER 1
const byte    encC = 20; //ENCODER 2
const byte    encD = 21; //ENCODER 2
const byte    encE = 18; //ENCODER 3
const byte    encF = 19; //ENCODER 3

const byte    fincarrera = 9;

/////////////////////////////////////VARIABLES ENCODER Y CMD////////////////////////////////

byte  estado = 0;
volatile long contadorBr = 0;
volatile long contadorAntBr = 0;
volatile long contadorZ = 0;
byte          antBr = 0, actBr = 0;
byte          antAntBr = 0, actAntBr = 0;
byte          antZ = 0, actZ = 0;
byte cmd = 0;
int flags=0;

//////////////////////////////////////VARIABLES CÁLCULO DEL PID//////////////////////////////////

double setpointgrados = 0.0;
//double setpoint = 0.0;
double setpointBr = 0.0, inputBr = 0.0, outputBr = 0.0; ///SETPOINT ES EL ANGULO QUE VAMOS A IR AUMENTANDO POCO A POCO HASTA LLEGAR AL ANGULO FINAL DEL BR
double setpointAntBr = 0.0, inputAntBr = 0.0, outputAntBr = 0.0; ///SETPOINT ES EL ANGULO QUE VAMOS A IR AUMENTANDO POCO A POCO HASTA LLEGAR AL ANGULO FINAL DEL BR
double setpointZ = 0.0, inputZ = 0.0, outputZ = 0.0; ///SETPOINT ES EL ANGULO QUE VAMOS A IR AUMENTANDO POCO A POCO HASTA LLEGAR AL ANGULO FINAL DEL BR

double kp = 0.0, ki = 0.0, kd = 0.0;
double outmax = 0.0, outmin = 0.0;
unsigned int tmp = 0;
float error = 5;          ///ERROR QUE QUEREMOS COMETER EN LA POSICION DE LOS MOTORES EN CLICKS

float angFinalBr = 0;       ///ANGULO FINAL DEL MOTOR QUE LE PASA LA CINEMATICA
float angFinalAntBr = 0;   // ANGULO FINAL CADA MOTOR (LOS DOS ROT, Y EL PRISM // LUEGO LES PASARÉ LOS CLICKS A DONDE TIENEN QUE IR
float angFinalZ = 0;

float angActualBr = 0;       ///ANGULO DEL MOTOR AL QUE TENGO QUE IR, IRE AUMENTANDOLO POCO A POCO PARA QUE AL FINAL EL ANGACTUAL LLEGUE AL ANGFINAL
float angActualAntBr = 0;   
float angActualZ = 0;

PID motorBr(&inputBr, &outputBr, &setpointBr, 0.0, 0.0, 0.0, DIRECT);
PID motorAntBr(&inputAntBr, &outputAntBr, &setpointAntBr, 0.0, 0.0, 0.0, DIRECT);
PID motorZ(&inputZ, &outputZ, &setpointZ, 0.0, 0.0, 0.0, DIRECT);

//////////////////////////////////////VARIABLES PARA EL CALCULO DE LA CINEMATICA//////////////////////////////

float pi = atan2(1, 1) * 4;
float rad = pi / 180;
float grad = 180 / pi;
int acabar = 0;


void setup() {
  ///INICIALIZACIÓN DE LOS PINES DEL ARDUINO Y DEL SERIAL///////////////////////
  Serial.begin(115200);
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  pinMode(PWMA, OUTPUT); // MOTOR 1
  pinMode(PWMB, OUTPUT); // MOTOR 1
  pinMode(PWMC, OUTPUT); // MOTOR 2
  pinMode(PWMD, OUTPUT); // MOTOR 2
  pinMode(fincarrera, INPUT);
  digitalWrite(PWMA, LOW);
  digitalWrite(PWMB, LOW);
  digitalWrite(PWMC, LOW);
  digitalWrite(PWMD, LOW);

  TCCR0B = TCCR0B & B11111000 | 1;    ///REDUCE EL RUIDO DE LOS MOTORES

  /////////////////////////////CREAMOS LAS INTERRUCCIONES DE LOS ENCODERS///////////////////

  attachInterrupt(digitalPinToInterrupt(encA), encoderBr, CHANGE);        /*ESTO ES PARA UN MOTOR DEL BRAZO*/
  attachInterrupt(digitalPinToInterrupt(encB), encoderBr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encC), encoderAntBr, CHANGE);        /*ESTO ES PARA UN MOTOR DEL ANTEBRAZO*/
  attachInterrupt(digitalPinToInterrupt(encD), encoderAntBr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encE), encoderZ, CHANGE);        /*ESTO ES PARA UN MOTOR DEL EJE Z*/
  attachInterrupt(digitalPinToInterrupt(encF), encoderZ, CHANGE);

  ////////////////////////////LE DAMOS UN VALOR A LAS VARIABLES DEL PID CUANDO ARRANQUEMOS EL ARDUINO//////////////////////////////////
  outmax = 250;         /*ES LA VELOCIDAD MAXIMA A LA QUE IRÁ EL MOTOR*/
  outmin = -outmax;     /*VALOR ENTRE 0 Y 255*/

  tmp = 25;           /*TIEMPO QUE REFRESCAS LOS VALORES EL PID EN "ms"*/

  kp = 12.0;          /*constantes del PID*//*LO SUYO SERIA CREAR UNAS VARIABLES DIFERENTES PARA CADA MOTOR YA QUE CADA UNO TENDRA SUS CONSTANTES*/
  ki = 0.8;
  kd = 4.0;

  motorBr.SetSampleTime(tmp);                 /*METEMOS LOS VALORES ANTERIORES AL MOTOR DEL BRAZO*/
  motorBr.SetOutputLimits(outmin, outmax);
  motorBr.SetTunings(kp, ki, kd);
  motorBr.SetMode(AUTOMATIC);
  
  motorAntBr.SetSampleTime(tmp);                 /*METEMOS LOS VALORES ANTERIORES AL MOTOR DEL ANTEBRAZO*/
  motorAntBr.SetOutputLimits(outmin, outmax);
  motorAntBr.SetTunings(kp, ki, kd);
  motorAntBr.SetMode(AUTOMATIC);
  
  motorZ.SetSampleTime(tmp);                 /*METEMOS LOS VALORES ANTERIORES AL MOTOR DEL EJE Z*/
  motorZ.SetOutputLimits(outmin, outmax);
  motorZ.SetTunings(kp, ki, kd);
  motorZ.SetMode(AUTOMATIC);


  setpointBr = 0;                   /*LE DECIMOS QUE EMPIECE EN 0 TODOS LOS SETPOINTS DE LOS MOTORES LOS 3*/
  setpointAntBr = 0;
//  setpoint = 0;

  /*do{                            //LLEVAMOS LOS MOTORES A LOS FINALES DE CARRERA
     analogWrite(PWMA,50);
     analogWrite(PWMC,50);
     deteccion();
    }while(estado==1);
    digitalWrite(PWMA,0);
    digitalWrite(PWMC,0);
    contador=0;
    contador2=contador;
    setpoint= (double)contador;
    }*/
}

void loop() {
  int N = 3;  // Numero de movimientos de cada secuencia
  float longBr = 150, longAntBr = 150, hip, angBr, angAntBr, alpha, beta, altMin = 0; //Cambiar dimensiones del brazo y altura minima a donde puede bajar el brazo respecto de la base


  float vx[N], vy[N], vz[N];   // vx vy y vz son las coordenadas de los putos que tiene la secuencia v
  vx[0] = 120, vy[0] = 20, vz[0] = 50; //Coordenadas finales del brazo en movimeinto 1
  vx[1] = 10, vy[1] = 40, vz[1] = 20; //Coordenadas finales del brazo en movimeinto 2
  vx[2] = 50, vy[2] = 50, vz[2] = 30; //Coordenadas finales del brazo en movimeinto 3



  float distPunto; //Distancia desde el origen hasta el punto final del brazo que se pide
  int i;


  ////..... FOR PARA VER LA SECUENCIA DE PUNTOS QUE LE ORDENO .....////
  if (acabar == 0)
  {
    for (i = 0; i < N; i++)
    {
      distPunto = dist(vx[i], vy[i]);
      if ((distPunto <= longBr + longAntBr) && distPunto >= longBr - longAntBr && vx[i] >= 0 && vy[i] >= 0 && vz[i] >= altMin)
      {
        hip = dist(vx[i], vy[i]);

        alpha = atan2(vy[i], vx[i]);
        alpha = alpha * grad;

        beta = acos((pow(longBr, 2) + pow(hip, 2) - pow(longAntBr, 2)) / (2 * longAntBr * hip));
        beta = beta * grad;

        angBr = alpha + beta;

        angAntBr = acos((pow(longBr, 2) + pow(longAntBr, 2) - pow(hip, 2)) / (2 * longBr * longAntBr));
        angAntBr = angAntBr * grad;


        Serial.print("Prismatico: ");
        Serial.println(vz[i]);
        Serial.print("Rotacional 1: ");
        Serial.println(angBr);
        Serial.print("Rotacional 2: ");
        Serial.println(angAntBr);

        motores(vz[i], angBr, angAntBr);
      }
      else
      {
        Serial.println("No es posible llegar al punto indicado");
      }
    }
    acabar = 1;
  }


}
/////////////////////////////////////////Cálculo de hipotenusa////////////////////////////////////
float dist(float x, float y)
{
  float d;
  d = sqrt((pow(x, 2)) + (pow(y, 2)));
  //cout << "Hip es: " << hip<<endl;
  return d;
}


///////////////////////////////////////////Conversión de grados a clicks///////////////////////////////

int ClicksRot(float ang)
{
  int n;
  n = (ang * 8350) / 360;
  return n;
}

int ClicksPris(float h)
{
  int n;
  n = (h*2) /3 ;    /*numero de clisk en cm medidos prueba donde esta el 2*//*numero cm medidos en la prueba donde esta el 3*/
  return n;         /*es para ver si compila*/
}

/////////////////////////////////////////////////////////////Función para mover motores//////////////////////////////////////////////
void motores(float z, float angBr, float angAntBr)
{

  ////  usar parametros z (cm) para prismatico, angBr(grados) para primer rot, angAntBr(grados) para segundo rot
  /// usar funcion ClicksRot para pasar los angulos rot 1 y rot 2 a clicks
  /// usar funcion ClicksPris para pasar la latura de prismatico a clicks
  //// ......... FUNCION DE MOTORES LUIS YA CON CLICKS ........////

  
  angFinalBr = ClicksRot(angBr);     /*igualo el valor que me pasas para ajustarlo a mi funcion*/
  angFinalAntBr=ClicksRot(angAntBr); // Guardo todos los angulo tranformados en clicks en variables que ire aumentando poco a poco desde el punto de clicks anterior 
  angFinalZ=ClicksPris(z);
  
  

  while((angFinalBr - inputBr > -error && angFinalBr - inputBr < error)&&(angFinalAntBr-inputAntBr>-error && angFinalAntBr-inputAntBr<error)&&(angFinalZ-error>-error&&angFinalZ-error<error))
  {

///////////////////////////////////Movimiento del MotorZ//////////////////////////////////////////////
     
      if((angFinalZ - inputZ < -error && angFinalZ - inputZ > error))   //Comprobamos si el motorBr ha llegado a su sitio
      { 
        inputZ = (double)contadorZ;
        while (!motorZ.Compute());
          if ((setpointZ - contadorZ > -error && setpointZ - contadorZ < error)) //poner un intervalo del error que se quiera cometer
          {
            digitalWrite(PWME, LOW);
            digitalWrite(PWMF, LOW);
            setpointZ += error;
    
            if (setpointZ > angActualZ)
            {
              setpointZ = angActualZ;
            }
    
          }
          else
          {
            if (outputZ > 0.0)
            {
              digitalWrite(PWMF, LOW);
              analogWrite(PWME, abs(outputZ));
      
            }
            else                              // Mueve el motor hacia  atrás   con el PWM correspondiente a su posición.
            {
              digitalWrite(PWME, LOW);        // Pone a 0 el primer pin del puente en H.
              analogWrite(PWMF, abs(outputZ)); // Por el segundo pin sale la señal PWM.
      
            }
          }
      }
      else
      {
        digitalWrite(PWME, LOW);
        digitalWrite(PWMF, LOW);
      }
      
    
////////////////////////Movimiento del motorBr//////////////////////////////////////////////////////////////////////////////////////////////

      if((angFinalBr - inputBr < -error && angFinalBr - inputBr > error))   //Comprobamos si el motorBr ha llegado a su sitio
      {                                                                     //si no ha llegado entramos en el if, si llega para el motorBr
        inputBr = (double)contadorBr;               
        while (!motorBr.Compute());
             
          if ((setpointBr - contadorBr > -error && setpointBr - contadorBr < error)) //poner un intervalo del error que se quiera cometer
          {
            digitalWrite(PWMA, LOW);
            digitalWrite(PWMB, LOW);
            setpointBr += error;
    
            if (setpointBr > angActualBr)
            {
              setpointBr = angActualBr;
            }
    
          }
          else
          {
            if (outputBr > 0.0)
            {
              digitalWrite(PWMB, LOW);
              analogWrite(PWMA, abs(outputBr));
      
            }
            else                              // Mueve el motor hacia  atrás   con el PWM correspondiente a su posición.
            {
              digitalWrite(PWMA, LOW);        // Pone a 0 el primer pin del puente en H.
              analogWrite(PWMB, abs(outputBr)); // Por el segundo pin sale la señal PWM.
      
            }
          }
      }
      else                                            
      {
        digitalWrite(PWMA, LOW);
        digitalWrite(PWMB, LOW);
      }

////////////////////////////////////////////////////Movimiento del MotorAntBr////////////////////////////////////////
      
      if((angFinalAntBr - inputAntBr < -error && angFinalAntBr - inputAntBr > error))   //Comprobamos si el motorAntBr ha llegado a su sitio
      {                                                                     //si no ha llegado entramos en el if, si llega para el motorBr
        inputAntBr = (double)contadorAntBr;
        while (!motorAntBr.Compute());
          if ((setpointAntBr - contadorAntBr > -error && setpointAntBr - contadorAntBr < error)) //poner un intervalo del error que se quiera cometer
          {
            digitalWrite(PWMC, LOW);
            digitalWrite(PWMD, LOW);
            setpointAntBr += error;
    
            if (setpointAntBr > angActualAntBr)
            {
              setpointAntBr = angActualAntBr;
            }
    
          }
          else
          {
            if (outputAntBr > 0.0)
            {
              digitalWrite(PWMD, LOW);
              analogWrite(PWMC, abs(outputAntBr));
      
            }
            else                              // Mueve el motor hacia  atrás   con el PWM correspondiente a su posición.
            {
              digitalWrite(PWMC, LOW);        // Pone a 0 el primer pin del puente en H.
              analogWrite(PWMD, abs(outputAntBr)); // Por el segundo pin sale la señal PWM.
      
            }
          }
      }
      else                                            
      {
        digitalWrite(PWMC, LOW);
        digitalWrite(PWMD, LOW);
      }
  }/////////////////////////////////////////////Aqui termina el while que mueve los motores
  


          
      
      


  if (Serial.available() > 0)           // Comprueba si ha recibido algún dato por el terminal serie.
  {
    cmd = 0;                            // Por seguridad "limpiamos" cmd.
    cmd = Serial.read();                // "cmd" guarda el byte recibido.
    if (cmd > 31)
    {
      /* byte flags = 0;                                     // Borramos la bandera que decide lo que hay que imprimir.
        if (cmd >  'Z') cmd -= 32;                          // Si una letra entra en minúscula la covierte en mayúscula.
        if (cmd == 'W') { setpoint += 5.0;     flags = 2; } // Si (por ejemplo) es la letra 'W' mueve 5 pasos hacia delante. Estos son movimientos relativos.
        if (cmd == 'Q') { setpoint -= 5.0;     flags = 2; } // Aquí son esos 5 pasos pero hacia atrás si se pulsa la letra 'Q'.
        if (cmd == 'S') { setpoint += 400.0;   flags = 2; } // Se repite lo mismo en el resto de las teclas.
        if (cmd == 'A') { setpoint -= 400.0;   flags = 2; }
        if (cmd == 'X') { setpoint += 5000.0;  flags = 2; }
        if (cmd == 'Z') { setpoint -= 5000.0;  flags = 2; }
        if (cmd == '2') { setpoint += 12000.0; flags = 2; }
        if (cmd == '1') { setpoint -= 12000.0; flags = 2; }
        // Decodificador para modificar las constantes PID.*/
      switch (cmd)                                                                           // Si ponemos en el terminal serie, por ejemplo "p2.5 i0.5 d40" y pulsas enter  tomará esos valores y los cargará en kp, ki y kd.
      { // También se puede poner individualmente, por ejemplo "p5.5", sólo cambiará el parámetro kp, los mismo si son de dos en dos.
        case 'P': kp  = Serial.parseFloat(); motorBr.SetTunings(kp, ki, kd); flags = 1; break; // Carga las constantes y presenta en el terminal serie los valores de las variables que hayan sido modificadas.
        case 'I': ki  = Serial.parseFloat(); motorBr.SetTunings(kp, ki, kd); flags = 1; break;
        case 'D': kd  = Serial.parseFloat(); motorBr.SetTunings(kp, ki, kd); flags = 1; break;
        case 'T': tmp = Serial.parseInt();   motorBr.SetSampleTime(tmp);     flags = 1; break;
        case 'G': setpointgrados = Serial.parseFloat();                    flags = 2; break;  // Esta línea permite introducir una posición absoluta. Ex: g13360 (y luego enter) e irá a esa posición.
        case 'K':                                                          flags = 3; break;
      }

      setpointBr = map(setpointgrados, 0, 360, 0, 8350);
      imprimir(flags);
    }
  }
}

void encoderBr()      //funcion que lee los clicks del encoder de los motores
{
  antBr = actBr;                        // Guardamos el valor 'act' en 'ant' para convertirlo en pasado.

  if (digitalRead(encA) == 1) bitSet(actBr, 0); else bitClear(actBr, 0); // Seteamos los dos primeros bits
  //  de la variable 'act' con
  if (digitalRead(encB) == 1) bitSet(actBr, 1); else bitClear(actBr, 1); // el valor de este instante, como
  //  un número de dos bits.

  if (antBr == 3 && actBr == 1) contadorBr++;
  if (antBr == 1 && actBr == 0) contadorBr++;
  if (antBr == 0 && actBr == 2) contadorBr++;
  if (antBr == 2 && actBr == 3) contadorBr++;

  if (antBr == 1 && actBr == 3) contadorBr--;
  if (antBr == 0 && actBr == 1) contadorBr--;
  if (antBr == 2 && actBr == 0) contadorBr--;
  if (antBr == 3 && actBr == 2) contadorBr--;
}

void encoderAntBr()      //funcion que lee los clicks del encoder de los motores
{
  antAntBr = actAntBr;                        // Guardamos el valor 'act' en 'ant' para convertirlo en pasado.

  if (digitalRead(encC) == 1) bitSet(actAntBr, 0); else bitClear(actAntBr, 0); // Seteamos los dos primeros bits
  //  de la variable 'act' con
  if (digitalRead(encD) == 1) bitSet(actAntBr, 1); else bitClear(actAntBr, 1); // el valor de este instante, como
  //  un número de dos bits.

  if (antAntBr == 3 && actAntBr == 1) contadorAntBr++;
  if (antAntBr == 1 && actAntBr == 0) contadorAntBr++;
  if (antAntBr == 0 && actAntBr == 2) contadorAntBr++;
  if (antAntBr == 2 && actAntBr == 3) contadorAntBr++;

  if (antAntBr == 1 && actAntBr == 3) contadorAntBr--;
  if (antAntBr == 0 && actAntBr == 1) contadorAntBr--;
  if (antAntBr == 2 && actAntBr == 0) contadorAntBr--;
  if (antAntBr == 3 && actAntBr == 2) contadorAntBr--;
}


void encoderZ()      //funcion que lee los clicks del encoder de los motores
{
  antZ = actZ;                        // Guardamos el valor 'act' en 'ant' para convertirlo en pasado.

  if (digitalRead(encE) == 1) bitSet(actZ, 0); else bitClear(actZ, 0); // Seteamos los dos primeros bits
  //  de la variable 'act' con
  if (digitalRead(encF) == 1) bitSet(actZ, 1); else bitClear(actZ, 1); // el valor de este instante, como
  //  un número de dos bits.

  if (antZ == 3 && actZ == 1) contadorZ++;
  if (antZ == 1 && actZ == 0) contadorZ++;
  if (antZ == 0 && actZ == 2) contadorZ++;
  if (antZ == 2 && actZ == 3) contadorZ++;

  if (antZ == 1 && actZ == 3) contadorZ--;
  if (antZ == 0 && actZ == 1) contadorZ--;
  if (antZ == 2 && actZ == 0) contadorZ--;
  if (antZ == 3 && actZ == 2) contadorZ--;
}


void deteccion()        //funcion para detectar los finales de carrera
{
  estado = digitalRead(fincarrera);
}

void imprimir(byte flag) // Imprime en el terminal serie los datos de las contantes PID, tiempo de muestreo y posición. En los demás casos sólo imprime la posición del motor.
{
  if ((flag == 1) || (flag == 3))
  {
    Serial.print("KP=");     Serial.print(kp);
    Serial.print(" KI=");    Serial.print(ki);
    Serial.print(" KD=");    Serial.print(kd);
    Serial.print(" Time=");  Serial.println(tmp);
  }
  if ((flag == 2) || (flag == 3))
  {
    Serial.print("Grados:");
    Serial.println((float)setpointgrados);
    Serial.print("Posicion:");
    Serial.println((float)setpointBr);
  }
}
