#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define sensorGotas (1<<PB0) //Entrada Digital simulada por um pushbottom com Interrupção
#define sensorUltrassonico 0 //Entrada Analógica simulada por um Trimpot Multivoltas

#define rotina (1<<PD4)

#define motorRotativo (1<<PD6) //Motor DC controlado por um PWM

#define buzzer (1<<PD5) //Buzzer Digital

#define volume 0.05 //volume em ml de cada gota individualmente
#define volumeMaximo 7.5 //Se o motor estiver em velocidade máxima, pode chegar até 7.5 ml por minuto

//Variáveis em geral
unsigned long int quantidadeGotas = 0; //Quantidade total de gotas caídas
bool contaGotas; //habilita a contagem de gotas dentro do período definido
bool auxConfig = true; //auxiliar para travar a execução do código até que se entre com os valores da rotina do conta gotas

float volumeUsuario; //Volume definido pelo usuário
unsigned int tempoUsuario; //Tempo de execução da rotina definida pelo usuário

float varAux = 0; //Variável auxiliar para ajuste da velocidade do motor
int contagem = 0; //auxiliar para contagem de tempo da rotina

unsigned int minutos = 0; //Contagem de minutos
short int segundos = 0; //contagem de segundos

//Variáveis de uso da comunicação serial
char RX_buffer[32]; 
char RX_index = 0;
char old_rx_hs[32]; // Buffer para estado anterior do RX
char TX_str[10]; //String auxiliar para saída de dados

//Funções para uso da comunicação serial
void UART_init(int baud){ // Função de configuração do UART
  int MYUBRR = 16000000 / 16 / baud - 1;
  UBRR0H = (unsigned char)(MYUBRR >> 8);
  UBRR0L = (unsigned char)(MYUBRR);
  UCSR0C = (0 << USBS0) | (1 << UCSZ00) | (1 << UCSZ01);
  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
}

void UART_send(char *TX_buffer){ // Função para envio de dados via UART
    while (*TX_buffer != 0){
      UDR0 = *TX_buffer;
      while (!(UCSR0A & (1 << UDRE0))){};
      TX_buffer++;
    }
}

void limpa_RX_buffer(void){ // Limpa o buffer de entrada e saída 
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

// Funções Auxiliares para teste e convesão de float para string (funções padrões para tal função)
void reverse(char* str, int len){
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}
 
// Converte um inteiro dado x para uma string str[]
// d é o número de dígitos necessários na saída
// Se d for maior do que o número de dígitos em x, então zeros são adicionados no início
int intToStr(int x, char str[], int d){
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }
 
    // Se o número de dígitos necessários for maior, adicionar zeros no início
    while (i < d)
        str[i++] = '0';
 
    reverse(str, i);
    str[i] = '\0';
    return i;
}
 
// Converte um número float/double em string
void ftoa(float n, char* res, int afterpoint){
    // Extrai somente a parte inteira do valor
    int ipart = (int)n;
 
    // Extrai somente a parte decimal do valor
    float fpart = n - (float)ipart;
 
    // Convere a parte inteira para string
    int i = intToStr(ipart, res, 0);
 
    // Verificando a opção de exibição após o ponto
    if (afterpoint != 0) {
        res[i] = '.'; // add dot
 
        // Obtém o valor da parte da fração até o número de pontos. O terceiro parâmetro é necessário para lidar com casos como 233.007
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
    volumeUsuario = atoi(RX_buffer);
    limpa_RX_buffer();
    auxConfig = true;

    UART_send("\nEntre com o Tempo de infusão em minutos:\n");
    while(auxConfig==true)
    _delay_ms(50);
    tempoUsuario = atoi(RX_buffer);
    limpa_RX_buffer();
    contaGotas = true;

    UART_send("\nRotina iniciada!\n");

    TCCR2B = (1<<CS21); //prescaler 8

    varAux = volumeUsuario/tempoUsuario;
    varAux = (varAux/volumeMaximo)*255;
    if(varAux > 255)
    varAux = 255;
    OCR0A = (int)(varAux);  
}

void calculaPercent(){  //Função para cálculo da porcentagem de erro

  float erroPercent; //Porcentagem de erro em relação a leitura/definição

  erroPercent = quantidadeGotas*volume;
  erroPercent = (erroPercent/tempoUsuario*1.0);
  erroPercent = ((erroPercent-(volumeUsuario/tempoUsuario*1.0))/(volumeUsuario/tempoUsuario*1.0))*100.0;
  
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

int main(){

  //Inicialização do UART com Baud Rate de 9600
  UART_init(9600);

  //Config I/O
  DDRD = motorRotativo+buzzer;
  PORTD &= ~(motorRotativo+buzzer);
  PORTD |= rotina;

  //Configuração PCINT Portal B - Pino PB0
  PCICR = (1<<PCIE0)|(1<<PCIE2);
  PCMSK0 = sensorGotas;
  PCMSK2 = rotina;

  //Configuração PWM
  TCCR0A = (1<<WGM01)|(1<<WGM00)|(1<<COM0A1);
  TCCR0B = (1<<CS02)|(1<<CS00); //PS 1024

  //Config leitura ADC
  ADMUX = (0 << REFS1) + (1 << REFS0); //Utiliza 5V como referência (1023)
	ADCSRA = (1 << ADEN) + (1 << ADPS2) + (1 << ADPS1) + (1 << ADPS0); //Habilita ADC e PS 128 (10 bits)
	ADCSRB = 0; //Conversão Única
  ADMUX = (ADMUX & 0xF8) | sensorUltrassonico; //Configura o pino PC0 para leitura ADC
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
      PORTD |= buzzer;
      OCR0A = 0;
      TCCR2B &= ~((1<<CS22)|(1<<CS21)|(1<<CS20));
    }

    else if(contaGotas == true){
      PORTD &= ~buzzer;
      OCR0A = (int)(varAux);
      TCCR2B = (1<<CS21);
    }
  }
}

//Interrupção Sensor Conta Gotas (BOTÃO PB0)
//Interrupção PCINT Para contagem de gotas

ISR(PCINT0_vect){
  
  if(((PINB & sensorGotas) == sensorGotas)){
    return;
  }

  else if(contaGotas == true){
    PCICR &= ~(1<<PCIE0);
    quantidadeGotas++;
    itoa(quantidadeGotas,TX_str,10);
    UART_send(TX_str);
    UART_send("\n");
  }
  PCICR |= (1<<PCIE0);  
}

ISR(PCINT2_vect){
  
  if(!((PIND & rotina) == rotina)){
    return;
  }
  else if(contaGotas == false){
    PCICR &= ~(1<<PCIE1);
    auxConfig=true;
  }
  PCICR |= (1<<PCIE1);
}

//Timer para contagem do tempo de execução da rotina 
//Interrupção do Timer para contagem do tempo e termino de Rotina
ISR(TIMER2_COMPA_vect){
	contagem++;
  if(contagem == 5000){
    contagem=0;
    segundos++;
    if(segundos == 60){
      segundos=0;
      minutos++;
      if(tempoUsuario == minutos){
        minutos = 0;
        varAux = 0;
        OCR0A = (int)(varAux);
        contaGotas = false;
        TCCR2B &= ~((1<<CS22)|(1<<CS21)|(1<<CS20)); //Timer desligado
        UART_send("\nRotina Encerrada!\n");
        calculaPercent();
      }
    }
  }
}

//Função ISR que salva um array de dados recebidos via UART 
//Interrupção de recepção de dados para salvar os dados recebidos
ISR(USART_RX_vect){ 

  RX_buffer[RX_index] = UDR0;
  RX_buffer[RX_index+1] = 0;
  RX_index++;

  auxConfig = false;
}
