#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define SGOTAS (1<<PB0)
#define CONFIGROTINA (1<<PD4)
#define SULTRA 0
#define MOTOR (1<<PD6)
#define BUZZER (1<<PD5) //BUZZER 5V com Oscilador + Config em GPIO DC
#define VOLUME 0.05
#define VOLMAX 7.5 //Volume máximo por minuto em ml para 100% da velocidade do motor

//Variáveis gerais de funcionamento do projeto ----------------------------------------------------------------------------------------------------------------
unsigned long int qntGotas = 0;
float volumeDef; //Volume definido pelo usuário
float auxVel = 0; //Auxiliar para ajuste da velocidade do motor
short int segundos = 0; //contagem de segundos
unsigned int minutos = 0; //Contagem de minutos
unsigned int minMax; //Tempo de execução da rotina definida pelo usuário
bool contaGotas; //habilita a contagem de gotas dentro do período definido
int contagem = 0;
bool auxConfig = true; //Booleano auxiliar para travar a execução do código até que se entre com os valores da rotina do conta gotas

//Variáveis de uso da comunicação serial -----------------------------------------------------------------------------------------------------------------------
char RX_buffer[32];
char RX_index = 0;
char old_rx_hs[32]; // Buffer para estado anterior do RX
char TX_str[10]; //String auxiliar para saída de dados

//Funções para uso da comunicação serial -----------------------------------------------------------------------------------------------------------------------
void UART_init(int baud){ // Função de configuração do UART --------------------

  int MYUBRR = 16000000 / 16 / baud - 1;
  UBRR0H = (unsigned char)(MYUBRR >> 8);
  UBRR0L = (unsigned char)(MYUBRR);
  UCSR0C = (0 << USBS0) | (1 << UCSZ00) | (1 << UCSZ01);
  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
}

void UART_send(char *TX_buffer){ // Função para envio de dados via UART --------------------

    while (*TX_buffer != 0){
      UDR0 = *TX_buffer;
      while (!(UCSR0A & (1 << UDRE0))){};
      TX_buffer++;
    }
}

void limpa_RX_buffer(void) // Limpa o buffer de entrada e saída --------------------
{
    unsigned char dummy;
    while (UCSR0A & (1 << RXC0)){
      dummy = UDR0;
    }
    RX_index = 0;

    for (int i = 0; i < 32; i++){
      old_rx_hs[i] = RX_buffer[i];
      RX_buffer[i] = 0;
    }
}

// Funções Auxiliares para teste --------------------------------------------------------------------------------------------------------------------------------
//https://www.geeksforgeeks.org/convert-floating-point-number-string/
void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}
 
// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }
 
    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';
 
    reverse(str, i);
    str[i] = '\0';
    return i;
}
 
// Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;
 
    // Extract floating part
    float fpart = n - (float)ipart;
 
    // convert integer part to string
    int i = intToStr(ipart, res, 0);
 
    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot
 
        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);
 
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

void config_rotina(){
    auxConfig = true;
    //Entrada dos valores de definição para a rotina
    UART_send("Entre com o volume:\n");
    while(auxConfig==true)
    _delay_ms(50);
    volumeDef = atoi(RX_buffer);
    limpa_RX_buffer();
    auxConfig = true;

    UART_send("\nEntre com o Tempo de infusão em minutos:\n");
    while(auxConfig==true)
    _delay_ms(50);
    minMax = atoi(RX_buffer);
    limpa_RX_buffer();
    contaGotas = true;

    UART_send("\nRotina iniciada!\n");

    TCCR2B = (1<<CS21); //prescaler 8

    auxVel = volumeDef/minMax;
    auxVel = (auxVel/VOLMAX)*255;
    if(auxVel > 255)
    auxVel = 255;
    OCR0A = (int)(auxVel);  
}

void calculaPercent(){  //--------------------------------------Função para cálculo da porcentagem de erro - FINALIZADO-----------------------------------------

  float erroPercent; //Porcentagem de erro em relação a leitura/definição

  erroPercent = qntGotas*VOLUME;
  erroPercent = (erroPercent/minMax*1.0);
  erroPercent = ((erroPercent-(volumeDef/minMax*1.0))/(volumeDef/minMax*1.0))*100.0;
  
  UART_send("\nErro de Porcentagem: ");
  
  if(erroPercent > 0){
    ftoa(erroPercent,TX_str,2);
    UART_send(TX_str);
    UART_send("% acima do esperado;\n");
  }
  else if(erroPercent < 0){
    ftoa((erroPercent*(-1)),TX_str,2);
    UART_send(TX_str);
    UART_send("% abaixo do esperado;\n");
  }
  else{
    UART_send("0% sem erros;\n");
  }
}

int main(){ //------------------------------------------------------------------Função int main()----------------------------------------------------------------

  //Inicialização do UART com Baud Rate de 9600
  UART_init(9600);

  //Config I/O
  DDRD = MOTOR+BUZZER;
  PORTD &= ~(MOTOR+BUZZER);
  PORTD |= CONFIGROTINA;

  //Configuração PCINT Portal B - Pino PB0
  PCICR = (1<<PCIE0)|(1<<PCIE2);
  PCMSK0 = SGOTAS;
  PCMSK2 = CONFIGROTINA;

  //Configuração PWM
  TCCR0A = (1<<WGM01)|(1<<WGM00)|(1<<COM0A1);
  TCCR0B = (1<<CS02)|(1<<CS00); //PS 1024

  //Config leitura ADC
  ADMUX = (0 << REFS1) + (1 << REFS0); //Utiliza 5V como referência (1023)
	ADCSRA = (1 << ADEN) + (1 << ADPS2) + (1 << ADPS1) + (1 << ADPS0); //Habilita ADC e PS 128 (10 bits)
	ADCSRB = 0; //Conversão Única
  ADMUX = (ADMUX & 0xF8) | SULTRA; //Configura o pino PC0 para leitura ADC
  DIDR0 = (1 << PC0); //Desabilita o PC0 como pino digital - Não obrigatório

  //Config Timer - Sem ligar (PS desativado)
  TCCR2A = (1<<WGM21);  //ctc
  TIMSK2 = (1<<OCIE2A); //Habilita interrupção Timer 2A
  OCR2A = 0b11000111; //199  100us para PS=8

  //Habilitando interrupção global
  sei();

  //Configuração inicial da rotina
  config_rotina();

  for(;;){

    if(auxConfig==true){
      config_rotina();
    }

    unsigned int leituraAD;
    float ultra; //valor de tensão do sensor ultrasônico
    
		ADCSRA |= (1 << ADSC); //Inicia a conversão
		while((ADCSRA & (1<<ADSC)) == (1<<ADSC)); //Esperar a conversão
		leituraAD = ADC;
		ultra = (leituraAD * 5) / 1023.0; //Cálculo da Tensão

    if((ultra < 3.75) && (contaGotas == true)){
      PORTD |= BUZZER;
      OCR0A = 0;
      TCCR2B &= ~((1<<CS22)|(1<<CS21)|(1<<CS20));
    }
    else if(contaGotas == true){
      PORTD &= ~BUZZER;
      OCR0A = (int)(auxVel);
      TCCR2B = (1<<CS21);
    }

  }
}

//Interrupção Sensor Conta Gotas (BOTÃO PB0) ------------------------------------Interrupção PCINT Para contagem de gotas - FINALIZADO---------------------------
ISR(PCINT0_vect){
  
  if(((PINB & SGOTAS) == SGOTAS)){
    return;
  }
  else if(contaGotas == true){
    PCICR &= ~(1<<PCIE0);
    qntGotas++;
    itoa(qntGotas,TX_str,10);
    UART_send(TX_str);
    UART_send("\n");
  }
  PCICR |= (1<<PCIE0);  
}

ISR(PCINT2_vect){
  
  if(!((PIND & CONFIGROTINA) == CONFIGROTINA)){
    return;
  }
  else if(contaGotas == false){
    PCICR &= ~(1<<PCIE1);
    auxConfig=true;
  }
  PCICR |= (1<<PCIE1);
}

//Timer para contagem do tempo de execução da rotina -------------Interrupção do Timer para contagem do tempo e termino de Rotina - FINALIZADO-------------------
ISR(TIMER2_COMPA_vect){
	contagem++;
  if(contagem == 5000){
    contagem=0;
    segundos++;
    if(segundos == 60){
      segundos=0;
      minutos++;
      if(minMax == minutos){
        minutos = 0;
        auxVel = 0;
        OCR0A = (int)(auxVel);
        contaGotas = false;
        TCCR2B &= ~((1<<CS22)|(1<<CS21)|(1<<CS20)); //Timer desligado
        UART_send("\nRotina Encerrada!\n");
        calculaPercent();
      }
    }
  }
}

//Função ISR que salva um array de dados recebidos via UART -------------Interrupção de recepção de dados para salvar os dados recebidos - FINALIZADO------------
ISR(USART_RX_vect){ 

  RX_buffer[RX_index] = UDR0;
  RX_buffer[RX_index+1] = 0;
  RX_index++;

  auxConfig = false;
}