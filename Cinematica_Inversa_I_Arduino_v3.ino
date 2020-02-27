#include <PID_v1.h>
//////////////////////////////////////PINOUT DEL BRAZO///////////////////////////////////////

const byte    encA = 2;
const byte    encB = 3;
const byte    PWMA = 5;
const byte    PWMB = 6;
const byte    PWMC = 7;
const byte    PWMD = 8;
const byte    encC = 20;
const byte    encD = 21;
const byte    fincarrera = 9;

/////////////////////////////////////VARIABLES ENCODER Y CMD////////////////////////////////

byte  estado = 0;
volatile long contador = 0;
volatile long contador2 = 0;
byte          ant = 0, act = 0;
byte          ant2 = 0, act2 = 0;
byte cmd = 0;
int flags=0;

//////////////////////////////////////VARIABLES CÁLCULO DEL PID//////////////////////////////////

double setpointgrados = 0.0, setpoint = 0.0, input = 0.0, output = 0.0; ///SETPOINT ES EL ANGULO QUE VAMOS A IR AUMENTANDO POCO A POCO HASTA LLEGAR AL ANGULO FINAL
double kp = 0.0, ki = 0.0, kd = 0.0;
double outmax = 0.0, outmin = 0.0;
unsigned int tmp = 0;
float error = 5;          ///ERROR QUE QUEREMOS COMETER EN LA POSICION DE LOS MOTORES EN CLICKS
float angfinal = 0;       ///ANGULO FINAL DEL MOTOR QUE LE PASA LA CINEMATICA

PID motor(&input, &output, &setpoint, 0.0, 0.0, 0.0, DIRECT);

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
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(fincarrera, INPUT);
  digitalWrite(PWMA, LOW);
  digitalWrite(PWMB, LOW);
  digitalWrite(PWMC, LOW);
  digitalWrite(PWMD, LOW);

  TCCR0B = TCCR0B & B11111000 | 1;    ///REDUCE EL RUIDO DE LOS MOTORES

  /////////////////////////////CREAMOS LAS INTERRUCCIONES DE LOS ENCODERS///////////////////

  attachInterrupt(digitalPinToInterrupt(encA), encoder, CHANGE);        /*ESTO ES PARA UN MOTOR*/
  attachInterrupt(digitalPinToInterrupt(encB), encoder, CHANGE);

  ////////////////////////////LE DAMOS UN VALOR A LAS VARIABLES DEL PID CUANDO ARRANQUEMOS EL ARDUINO//////////////////////////////////
  outmax = 250;         /*ES LA VELOCIDAD MAXIMA A LA QUE IRÁ EL MOTOR*/
  outmin = -outmax;     /*VALOR ENTRE 0 Y 255*/

  tmp = 25;           /*TIEMPO QUE REFRESCAS LOS VALORES EL PID EN "ms"*/

  kp = 12.0;          /*constantes del PID*/
  ki = 0.8;
  kd = 4.0;

  motor.SetSampleTime(tmp);                 /*METEMOS LOS VALORES ANTERIORES AL MOTOR*/
  motor.SetOutputLimits(outmin, outmax);
  motor.SetTunings(kp, ki, kd);
  motor.SetMode(AUTOMATIC);


  setpoint = 0;                     /*LE DECIMOS QUE EMPIECE EN 0*/

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
  int N = 3;
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

float dist(float x, float y)
{
  float d;
  d = sqrt((pow(x, 2)) + (pow(y, 2)));
  //cout << "Hip es: " << hip<<endl;
  return d;
}

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

void motores(float z, float angBr, float angAntBr)
{

  ////  usar parametros z (cm) para prismatico, angBr(grados) para primer rot, angAntBr(grados) para segundo rot
  /// usar funcion ClicksRot para pasar los angulos rot 1 y rot 2 a clicks
  /// usar funcion ClicksPris para pasar la latura de prismatico a clicks
  //// ......... FUNCION DE MOTORES LUIS YA CON CLICKS ........////

  /////////////////////////////////////de momento es el MOTOR BR habria que meter los demas en el while los hacemos mas adelante//////////////////////////

  angfinal = ClicksRot(angBr);     /*igualo el valor que me pasas para ajustarlo a mi funcion*/

  while ((angfinal - input > -error && angfinal - input < error))
  {
    input = (double)contador;
    while (!motor.Compute());
    if ((setpoint - contador > -error && setpoint - contador < error)) //poner un intervalo del error que se quiera cometer
    {
      digitalWrite(PWMA, LOW);
      digitalWrite(PWMB, LOW);
      digitalWrite(PWMC, LOW);
      digitalWrite(PWMD, LOW);
      setpoint += error;

      if (setpoint > angfinal)
      {
        setpoint = angfinal;
      }

    }
    else
    {
      if (output > 0.0)
      {
        digitalWrite(PWMB, LOW);
        analogWrite(PWMA, abs(output));

      }
      else                              // Mueve el motor hacia  atrás   con el PWM correspondiente a su posición.
      {
        digitalWrite(PWMA, LOW);        // Pone a 0 el primer pin del puente en H.
        analogWrite(PWMB, abs(output)); // Por el segundo pin sale la señal PWM.

      }
    }
  }

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
        case 'P': kp  = Serial.parseFloat(); motor.SetTunings(kp, ki, kd); flags = 1; break; // Carga las constantes y presenta en el terminal serie los valores de las variables que hayan sido modificadas.
        case 'I': ki  = Serial.parseFloat(); motor.SetTunings(kp, ki, kd); flags = 1; break;
        case 'D': kd  = Serial.parseFloat(); motor.SetTunings(kp, ki, kd); flags = 1; break;
        case 'T': tmp = Serial.parseInt();   motor.SetSampleTime(tmp);     flags = 1; break;
        case 'G': setpointgrados = Serial.parseFloat();                    flags = 2; break;  // Esta línea permite introducir una posición absoluta. Ex: g13360 (y luego enter) e irá a esa posición.
        case 'K':                                                          flags = 3; break;
      }

      setpoint = map(setpointgrados, 0, 360, 0, 8350);
      imprimir(flags);
    }
  }
}

void encoder()      //funcion que lee los clicks del encoder de los motores
{
  ant = act;                        // Guardamos el valor 'act' en 'ant' para convertirlo en pasado.

  if (digitalRead(encA) == 1) bitSet(act, 0); else bitClear(act, 0); // Seteamos los dos primeros bits
  //  de la variable 'act' con
  if (digitalRead(encB) == 1) bitSet(act, 1); else bitClear(act, 1); // el valor de este instante, como
  //  un número de dos bits.

  if (ant == 3 && act == 1) contador++;
  if (ant == 1 && act == 0) contador++;
  if (ant == 0 && act == 2) contador++;
  if (ant == 2 && act == 3) contador++;

  if (ant == 1 && act == 3) contador--;
  if (ant == 0 && act == 1) contador--;
  if (ant == 2 && act == 0) contador--;
  if (ant == 3 && act == 2) contador--;
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
    Serial.println((float)setpoint);
  }
}
