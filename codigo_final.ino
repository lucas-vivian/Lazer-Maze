/*https://circuitdigest.com/micronum_motorroller-projects/stepper-motor-num_motorrol-with-potentiometer-arduino */

/*tivemos problemas ao utilizar o botao do encoder dado que é inicialmente LOW e por isso substituimos
  por um push button comum para o disparo que segue a mesma logica dos outros*/

#include <Stepper.h> // Include the header file
#include <RotaryEncoder.h>
#include <Servo.h>
#include <GFButton.h>
#include <LiquidCrystal.h>

// change this to the number of steps on your motor
#define STEPS 32

// create an instance of the stepper class using the steps and pins
Stepper stepper1(STEPS, 8, 10, 9, 11);
Stepper stepper2(STEPS, 4, 6, 5, 7);
Stepper stepper3(STEPS, 30, 32, 31, 33);
Stepper stepper4(STEPS, 36, 38, 37, 39);
/*
  Stepper stepper4(STEPS, 46, 48, 47, 49);
  Stepper stepper5(STEPS, 14, 16, 15, 17);
  Stepper stepper6(STEPS, 50, 52, 51, 53);
  Stepper stepper7(STEPS, 34, 36, 35, 37);
*/

LiquidCrystal lcd(40, 41, 42, 43, 44, 45);

int vida = 1100;

int angulo1 = 0;

byte block[8] =
{
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

RotaryEncoder encoder(20, 21);

int novo_ang = 0; //nova movimentacao para reset pós vitória
int Pval = 0;
int posicaoAnterior = 0;
unsigned long instanteAnterior1 = 0; //tempo para a porta 1
unsigned long instanteAnterior2 = 0; //tempo para a porta 2
unsigned long instanteAnteriorLaser = 0;
unsigned long instanteAtual = 0;
unsigned long t_servo1 = 0, t_servo2 = 0;
int tempo1 = 1; //tempo recebido convertido
int tempo2 = 0; //tempo recebido convertido
int motor_lcd = 3; //numero de motores para exibir no display
bool estado_porta1 = 0; //porta 1 se encontra fechada
bool estado_porta2 = 0; //porta 2 se encontra fechada
bool flag_chave = false;
bool flag_start = false;
int posicao = 0; //posicao encoder

//definicao dos servo motores
int pinoDoServo1 = 23;
int pinoDoServo2 = 50;
Servo servo1;
Servo servo2;

int num_motor = 0;

//Stepper v_motors[] = {stepper1, stepper2, stepper3, stepper4, stepper5, stepper6, stepper7};
Stepper v_motors[] = {stepper1, stepper2, stepper3, stepper4};

GFButton botao_esq(53,E_GFBUTTON_PULLDOWN);
GFButton botao_dir(52,E_GFBUTTON_PULLDOWN);
GFButton disparo(3, E_GFBUTTON_PULLDOWN);
GFButton botao_start(14, E_GFBUTTON_PULLDOWN);

/*
  GFButton botao1(3);
*/
//int disparo = 60; //botao de disparo
//int botao_dir = 43; //anda com o num_motorador para a direita
//int botao_esq = 42; //anda com o num_motorador para a esquerda
int laser = 18; //declarando laser
//int ldrPin = 0; //LDR no pino analígico 8
//int ldrValor = 0; //Valor lido do LDR

//configurando LDR
int sensorPin_alvo = A0; // select the input pin for LDR
int sensorPin_vida_extra = A1;
int sensorValue = 0; // variable to store the value coming from the sensor
int sensorValue1 = 0;

void tickDoEncoder() {
  encoder.tick();
}

void setup() {
  Serial.begin(9600); //inicializa serial

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PRESSIONE AZUL");
  lcd.setCursor(0, 1);
  lcd.print("PARA INICIAR...");

  pinMode(laser, OUTPUT);

  //configuracao pinos encoder
  int origem1 = digitalPinToInterrupt(20);
  int origem2 = digitalPinToInterrupt(21);

  //set das velocidades maximas dos motores de passo
  v_motors[0].setSpeed(800);
  v_motors[1].setSpeed(800);
  v_motors[2].setSpeed(800);
  v_motors[3].setSpeed(800);
  /*
    v_motors[4].setSpeed(800);
    v_motors[5].setSpeed(800);
    v_motors[6].setSpeed(800);
  */

  attachInterrupt(origem1, tickDoEncoder, CHANGE);
  attachInterrupt(origem2, tickDoEncoder, CHANGE);

  servo1.attach(pinoDoServo1);
  servo2.attach(pinoDoServo2);

  digitalWrite(laser, LOW); //inicializa laser desligado

  disparo.setPressHandler(acende_laser);
  disparo.setReleaseHandler(apaga_laser);
  disparo.setHoldHandler(conta_vida);
  disparo.setHoldTime(150);
  botao_esq.setPressHandler(ante_position); //entra na funcao de reduzir o contador
  botao_dir.setPressHandler(prox_position); //entra na funcao de aumentar o contador
  botao_start.setHoldHandler(start);
  botao_start.setHoldTime(500);
}

void loop() {
  //Serial.println(digitalRead(53));
  if (Serial.available() > 0) {
      Serial.println("Entrou Serial");
      // lê do buffer o dado recebido:
      String texto = Serial.readString();
      texto.trim();
      if (texto != "") {
        bool comecaCom = texto.startsWith("Cada"); //a mensagem a ser recebida é: Cada t1 t2 m1
        if (comecaCom) {
          String trecho1 = texto.substring(5);
          String trecho2 = texto.substring(7);
          String trecho3 = texto.substring(9);
          Serial.println(trecho1);
          Serial.println(trecho2);
          Serial.println(trecho3);
          //Serial.println(trecho);
          tempo1 = trecho1.toInt();
          tempo2 = trecho2.toInt();
          motor_lcd = trecho3.toInt() - 1;
          //Serial.println(tempo1);
          //Serial.println(tempo2);
        }
      }
    }
    
  botao_start.process();
  
  if (flag_start) {
    botao_esq.process();
    botao_dir.process();
    disparo.process();
    
    //tela();

    posicao = encoder.getPosition();
    if (posicao != posicaoAnterior) { //condicao de movimentacao dos motores pelo encoder
      Serial.println(num_motor);
      if (posicao > Pval) {
        v_motors[num_motor].step(15);
      }
      if (posicao < Pval) {
        v_motors[num_motor].step(-15);
      }
    }
    Pval = posicao;
    posicaoAnterior = posicao;

    
    porta_servo(); //chamando funcao da porta automatica]

    sensorValue = analogRead(sensorPin_alvo); // read the value from the sensor
    sensorValue1 = analogRead(sensorPin_vida_extra);
    //Serial.println(sensorValue); //prints the values coming from the sensor on the screen
    if (sensorValue > 900) {
      //Serial.println("VITÓRIA!");
      //    flag_vitoria = 1;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("VITORIA");
      lcd.setCursor(0, 1);
      lcd.print("REINICIAR?");
      flag_start = false;
      reset();
    }
    if (sensorValue1 > 900) {
      //Serial.println("VITÓRIA!");
      //    flag_vitoria = 1;
      vida = 1100;
      tela();
    }
    /*
      if(vida_extra > 900){

      }
    */
    //condicao para abrir portal ao pegar chave
    /*
      ldrValor = analogRead(ldrPin); //O valor lido será entre 0 e 1023
      //Serial.println(ldrValor);
      if (ldrValor >= 170) { //checando se laser atingiu chave e abriu porta
        Serial.println("CHAVE");
        if (flag_chave == false) {
          flag_chave = true;
          for (int angulo2 = 0; angulo2 <= 90; angulo2 = angulo2 + 2) {
            servo2.write(angulo2);
            delay(15);
          }
        }
      }
    */
  }
}
void porta_servo() {
  if (millis() - instanteAnterior1 > 15) { //condicao para servo da primeira porta automatica
    //Serial.println(angulo1);
    if (estado_porta1 == false) { //false a porta se encontra aberta
      angulo1 = angulo1 + 2;
      servo1.write(angulo1);
      if (angulo1 >= 180) {
        if (millis() - t_servo2 > tempo1 * 1000) {
          estado_porta1 = true;
          t_servo1 = millis();
        }
      }
    }
    else {
      angulo1 = angulo1 - 2;
      servo1.write(angulo1);
      if (angulo1 <= 70) {
        if (millis() - t_servo1 > tempo1 * 1000) {
          estado_porta1 = false;
          t_servo2 = millis();
        }
      }
    }
    instanteAnterior1 = millis();
    /*
      if (millis() - instanteAnterior2 > tempo2 * 1000) {  //condicao para servo da segunda porta automatica
      if (estado_porta2 == false) {
        for (int angulo2 = 0; angulo2 <= 90; angulo2 = angulo2 + 2) {
          servo2.write(angulo2);
          delay(15);
        }
        estado_porta2 = true;
      }
      else {
        for (int angulo2 = 90; angulo2 >= 0; angulo2 = angulo2 - 2) {
          servo2.write(angulo2);
          delay(15);
        }
        estado_porta2 = false;
      }
      instanteAnterior2 = millis();
      }
    */
  }
}

void prox_position(GFButton& botao_dir) {
  //Serial.println("direita");
  num_motor = num_motor + 1;
  //Serial.println(num_motor);
  if (num_motor > motor_lcd) {
    num_motor = 0;
  }
  tela();
  //Serial.println(num_motor);
}

void ante_position(GFButton& botao_esq) {
  //Serial.println("esquerda");
  num_motor = num_motor - 1;
  if (num_motor < 0) {
    num_motor = motor_lcd;
  }
  tela();
  //Serial.println(num_motor);
}

void acende_laser(GFButton& disparo) {
  //Serial.println("DISPARO!");
  digitalWrite(laser, HIGH);
}

void apaga_laser(GFButton& disparo) {
  digitalWrite(laser, LOW);
}

void tela() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MOTOR:");
  lcd.print(num_motor+1);
  lcd.setCursor(0, 1);
  lcd.print("VIDA: ");
  lcd.createChar(0, block);
  lcd.setCursor(5, 1);
  for (int i = 0; i < int((vida/100)); i++) {
    lcd.write(byte(0));
  }
}

void conta_vida() {
  //Serial.println("contou");
  vida = vida - 10;
  if(vida%100==0){
  tela();}
  if (vida == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("FIM DE JOGO");
    lcd.setCursor(0, 1);
    lcd.print("REINICIAR?");
    flag_start = false;
    reset();
  }
}

void start(GFButton& botao_start) {
  Serial.println("start");
  vida=1100;
  tela();
  flag_start = true;
}

void reset() {
  for (int i = 0; i < 4; i++) {
    novo_ang = random(100, 300);
    v_motors[i].step(novo_ang);

  }
}
