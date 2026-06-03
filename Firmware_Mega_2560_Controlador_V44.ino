//Autor: Aziel de Fontes Melo
//Data: 12/08/2025, hora: 10:59
//Descrição: Este é o código que testa mudanças de ganho a partir da leitura serial do monitor srial
/* Atualização
   Dia: 17/07/2025
   Hora: 10:21
   Descrição: implementando controle de ganho multável
*/
/* Atualização
   Dia: 22/07/2025
   Hora: 12:35
   Descrição: implementando de time out na comunicação com o supervisório no momento de pedido da confirmação de leitura do código de barras
*/
/* Atualização
   Dia: 28/07/2025
   Hora: 14:35
   Descrição: implementando de time out na comunicação com o modulo contador no momento de pedido pela leitura hp10
*/
/* Atualização
   Dia: 30/07/2025
   Hora: 15:26 mudança do tipo de variável de contador no envio
*/
/* Atualização
   Dia: 31/07/2025
   Hora: 18:14 implementação de mudança de ganho automático
*/
/* Atualização
   Dia: 31/07/2025
   Hora: 18:14 implementado time_out co operiférico motor
*/
/* Atualização
   Dia: 22/04/2025
   Autor: Aziel
   Hora: 08:59 adaptação do código final para uso com o protótipo.
*/
/* Atualização
   Dia: 02/06/2026
   Autor: Aziel
   Hora: 13:27 implementação do versionamento via git e github.
*/

/* Atualização
   Dia: 03/06/2026
   Autor: Willian
   Hora: 10:50 implementação da leitura da densidade de corrente do led.
*/

//#S1%M1G3L03000P4Z05000Q2& --modo automático padrão

//sequência de comandos no modo automático:
//    #S1%M1G3L03000P4Z01000Q4& //define parametros                   passo 1
//    #S1%C1001&                //habilita inicio de processo
//    #S1%C0001&                //inicia processo                     passo 2
//    #S1%C0110&                //confirma leitura do códig de barras passo 3
//    #S1%C1000&                //zerar                               passo 4
//    #S1%C0100&                //descarte ok                         passo 5
//OU  #S1%C0101&                //descarte não ok                     passo 5
// volta para passo 3
//sequência de comandos no modo automático com botão:
//    #S1%M1G3L30000P4Z30000Q2& //define parametros                   passo 1
//    #S1%C1001&                //habilita botão                      passo 2
//    APERTAR BOTÃO             //inicia processo                     passo 3
//    #S1%C0110&                //confirma leitura do códig de barras passo 4
//    #S1%C1000&                //zerar                               passo 5
//    #S1%C0100&                //descarte ok                         passo 6
//OU  #S1%C0101&                //descarte não ok                     passo 6
// volta para passo 3

//Como usar os comandos para testes SIMPLES
// autoteste -> escreva #S1%M3G3L03000P2Z03000Q4& e depois #S1%C0011&
// Leitura led -> escreva #S1%M1G3L06000P4Z01000Q4& e depois #S1%SC1001&
// Fazer zeramento -> escreva #S1%M3G3L60000P2Z03000Q4& e depois #S1%SC1011&
// Parar leitura #S1%C1010&

#include <SPI.h>

#define CS 53  // MCP41010 chip select - digital potentiometer.

//****************************************************
// pinos LED estimulacao
#define selecao_led 9       // lig/des LED
#define liga_desliga_led 7  // lig/des LED
#define Aled 3              // corrente (MSB)
#define Bled 5              // corrente (LSB)
//****************************************************

#include "exibir_lcd.h"    //tem imbutido todas as funções que escrevem algo na tela
#include "serial_event.h"  //tem a função que lê a porta serial
//#include <TimerThree.h>


int contador_saturacao = 0;
//*****************************************************************
int taxaAquisicao = 100;  // base de tempo (ms)
int numeroCanais = 0;     // numero de leituras (canal)
//*****************************************************************
int estado = 0;
//*****************************************************************
//  Variaveis protocolo de comunicacaoString
//  Instruções que vão do supervisório para o mega
String Stand_By = "#S1%C0000&";            //Stand-By
String iniciar = "#S1%C0001&";             //inicia o processo automático
String parar = "#S1%C0010&";               //parar
String autoteste = "#S1%C0011&";           //Auto-Teste
String dosimetro_ok = "#S1%C0100&";        //Descarte OK
String dosimetro_n_ok = "#S1%C0101&";      //Descarte não OK
String codigo_lido = "#S1%C0110&";         //Secesso ao ler codigo de barras dosímetro
String ler = "#S1%C0111&";                 //ler dosímetro
String zerar = "#S1%C1000&";               //zerar dosímetro
String habilitar_botao = "#S1%C1001&";     //equipamento
String desabilitar_botao = "#S1%C1010&";   //equipamento
String iniciar_modo_zerar = "#S1%C1011&";  //zerar dosímetro
//  Instruções que pulam a mecânica
String SUDO_leitura = "#S1%SC1001&";  //Leitura sem passar pela mecânica
String SUDO_stop = "#S1%SC1010&";     //para a leitura e desliga led sem passar pela mecânica
String SUDO_zerar = "#S1%SC1011&";    //Liga o led de zeramento sem passar pela mecânica
String SUDO_ligaLed = "#S1%SC1100&";  //Liga led de leitura sem passar pela mecânica
//  Informações que vão do mega para o supervisório
String torre_vazia = "#L1%T0000000&";
String standby = "#L1%I0000000&";
String buscando_origem = "#L1%I0000001&";
String remo_dos_torre = "#L1%I0000010&";
String lendo_dos = "#L1%I0000011&";
String lendo_dose = "#L1%I0000100&";  //--
String fim_leit = "#L1%I0000101&";
String zerando = "#L1%I0000110&";
String relendo = "#L1%I0000111&";  //--
String fim_releit = "#L1%I0001000&";
String ejetando = "#L1%I0001001&";
String dos_descartado = "#L1%I0001010&";
String fim_torre = "#L1%I0001011&";
String travamento = "#L1%I0001100&";
String parado = "#L1%I0001101&";
String botao_start = "#L1%I0001110&";
String botao_stop = "#L1%I0001111&";
String fim_hp10 = "#L1%I0010000&";   //--
String fim_hp007 = "#L1%I0010001&";  //--
String time_out_super_p = "#L1%I0010010&";
String id_maquina = "#L1%R3001A01&";  // ESPECÍFICO PARA CADA MÁQUINA INDIVIDUAL PRODUZIDA, NOME COMPOSTO DO MODELO "3001A" E NS "01"
//Palavras de informação do motor travado
String motores_bem = "#L1%M0000000&";
String mtr_torre_travado = "#L1%M0000001&";
String mtr_pista_travado = "#L1%M0000010&";
String mtr_leit_travado = "#L1%M0000100&";
//  Instruções que vão do mega para o controlador dos motores
String origem = "#C1%C000&";                   //posição origem
String dos_fora_torre = "#C1%C001&";           //posição HP10
String HP10 = "#C1%C010&";                     //posição HP10
String HP07 = "#C1%C011&";                     //posição HP07
String zeramento = "#C1%C100&";                //posição de zeramento
String descarte_ok = "#C1%C101&";              //posição descarte ok
String descarte_n_ok = "#C1%C110&";            //posição descarte não ok
String max_corrente = "#C1%C111&";             //pede a maxima corrente alcançada durante autoteste
String descrt_n_ok_desacoplado = "#C1%C112&";  //pede a maxima corrente alcançada durante autoteste
//  Informações que vão do controlador dos motores para o mega
String sucesso_pos = "#PM%00&";        //sucesso em alcançar a posição
String falha_pos = "#PM%01&";          //falha em alcançar a posição
String sem_dosimetro = "#PM%10&";      //sem dosimetro
String com_dosimetro = "#PM%11&";      //com dosimetro
String falha_motor_torre = "#PM%MT&";  //falha em alcançar a posição
String falha_motor_pista = "#PM%MP&";  //falha em alcançar a posição
String falha_motor_leit = "#PM%ML&";   //falha em alcançar a posição
boolean f_p_sucesso_pos = false;
boolean f_p_falha_pos = false;
boolean f_sem_dosimetro = false;
boolean f_com_dosimetro = false;
boolean f_mtr_torre_travado = false;
boolean f_mtr_pista_travado = false;
boolean f_mtr_leit_travado = false;
//*****************************************************************
//Variaveis de parâmetros
int dado_int = 0;
String tempoLeituraAux = "";
long int tempoLeitura = 0;  // tempo de leitura (s)
String tipoLEDAux = "";
String potenciaLEDLAux = "";
int potenciaLEDL = 0;
String potenciaLEDZAux = "";
int potenciaLEDZ = 0;
String modoLEDAux = "";
int modoLED = 0;
char ganhoPmtAux = ' ';
int ganhoPmt = 0;
String zeramentoAux = "";
long int tempo_zeramento = 0;
int zeramento_ou_leitura = 1;  //seleciona  o led que vai se ligado
//*****************************************************************
//flags para palavras da comunicação serial
boolean flag_p_parar = false;
boolean flag_p_codigo_lido = false;
boolean flag_p_iniciar = false;
boolean flag_p_dosimetro_ok = false;
boolean flag_p_dosimetro_n_ok = false;
boolean flag_p_ler = false;
boolean flag_p_zerar = false;
boolean flag_p_autoteste = false;
boolean flag_p_iniciar_modo_zerar = false;
//flags para palavras SUDO da comunicação serial
boolean flag_p_SUDO_leitura = false;
boolean flag_p_SUDO_stop = false;
boolean flag_p_SUDO_zerar = false;
boolean flag_p_SUDO_ligaLed = false;
boolean flag_sudo = false;
//*****************************************************************
//variáveis de informaçoes do autoteste
int luz_de_referencia = 0;
unsigned long iled_leitura = 0;
unsigned long iled_zeramento = 0;
int n_leit_i_zeramento = 0;
//*****************************************************************
boolean flag_autoteste = false;
boolean flag_autoteste_fim = false;
boolean f_dos_mod_leit = false;     //flag que indica se já tem um dosímetro no modo de leitura
boolean flag_zerar = false;         //flag que indica que o modo manual foi acionado e que é necessário apenas zerar o dosímetro e não ler ele
boolean flag_zerado_1_vez = false;  //flag que indica que o dosímetro já foi zerado ao menos 1 vez
//*****************************************************************
int canal = 0;  //Contador de dados recebidos do contador de pulsos da fotomultiplicadora.
//*****************************************************************
//variáveis do funcionamento o botão
unsigned long debounce_bt_start_stop = 0;
boolean flag_botao = false;
boolean flag_botao_habilitado = false;
//*****************************************************************
unsigned int contador_falhas_mec = 0;
//*****************************************************************
boolean flag_saturacao_pmt_p4 = false;
boolean flag_saturacao_pmt_p3 = false;
double f_correcao_pmt = 1;
//*****************************************************************
unsigned long time_out_supervisorio = 0;
int contador_time_out_supervisorio = 0;
unsigned long time_out_mod_contador = 0;
int contador_time_out_mod_contador = 0;
unsigned long time_out_mod_motor = 0;
int contador_time_out_mod_motor = 0;
//*****************************************************************


void setup() {
  Serial.begin(115200);   //comunicação com PC
  Serial2.begin(115200);  //comunicação com o periférico leitura
  Serial1.begin(115200);  //comunicação com o periférico leitura na placa protótipo
  Serial3.begin(115200);  //comunicação com o periférico motor

  //Serial.println("oi");

  lcd.init();
  lcd.backlight();

  exibirEmpresa();

  exibirInicializando();

  lcd.createChar(1, lcd_custom_char_linhas_verticais);
  lcd.createChar(2, lcd_custom_char_linhas_verticais_linha_final);
  lcd.createChar(3, lcd_custom_char_barra_de_progresso_1_linha);
  lcd.createChar(4, lcd_custom_char_barra_de_progresso_2_linhas);
  lcd.createChar(5, lcd_custom_char_barra_de_progresso_3_linhas);
  lcd.createChar(6, lcd_custom_char_barra_de_progresso_4_linhas);
  lcd.createChar(7, lcd_custom_char_barra_de_progresso_5_linhas);
  lcd.home();

  pinMode(CS, OUTPUT);
  SPI.begin();  // Set pins as outputs for SPI hardware.
  //********************************************
  //definir_ganho_pmt(70); // melhor resultado
  //********************************************
  //==========================
  //Prepara o pino do botão
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), interrupcao_2, FALLING);
  //==========================
  //Prepara os pinos de configuração do led
  //codigo A=0 e B=0 (corente 25%)
  pinMode(liga_desliga_led, OUTPUT);
  pinMode(selecao_led, OUTPUT);
  pinMode(Aled, OUTPUT);
  pinMode(Bled, OUTPUT);
  digitalWrite(Aled, LOW);              // cogido A = 0
  digitalWrite(Bled, LOW);              // cogido B = 0
  digitalWrite(liga_desliga_led, LOW);  // desliga led
  digitalWrite(selecao_led, LOW);       // seleciona led de leitura
  //==========================
  //Descarte de qualque dosímetro que por ventura esteja dentro do módulo de leitura
  Serial3.print(descarte_n_ok);
  boolean descarte_inicial = true;
  /*while (descarte_inicial) { //fica preso enquanto a
    serial3Event();
    if (nova_palavra_3) {
      //Serial.print(palavra_3);
      if (palavra_3 == falha_pos) {
        //Serial.print(travamento);
        //exibirFalhaMecanica();
        descarte_inicial = false;
        nova_palavra_3 = false;
        palavra_3 = "";
        break;
      } else if (palavra_3 == sucesso_pos) {
        descarte_inicial = false;
        nova_palavra_3 = false;
        palavra_3 = "";
        break;
      } else {
        nova_palavra_3 = false;
        palavra_3 = "";
        //Caso haja falha de comunicação e a palavra entregue seja diferente de sucesso_pos ou descarte pos o mega repete o comando
        Serial3.print(descarte_n_ok);
      }
    }
  }*/
  //========================
  //exibirEquipamento();
  //exibirAutoteste();
  //exibirFalhaLuzDeRef();
  //exibirFalhaMecanica();
  //exibirAutoTesteOK();
  //atualizar_barra_de_status();
  //exibirProcessoFinalizado();

  flag_botao = false;
  Serial.print(id_maquina);
  delay(100);
  Serial.print("#L1%I0000000&");  //liberar botão start do supervisório
}

void loop() {
  //tratamento das palavras vindas pela serial
  serial2Event();
  serial1Event();  //comunicação com o contado na placa protótipo
  serial3Event();
  serialEvent();

  trata_palavra_supervisorio();  //Lê a palavra da serial e ativa a flag correspondente, se for parâmetros define-os e aplica-os

  trata_botao();

  trata_palavra_motor();

  switch (estado) {
    case 0:  //Standby
      contador_falhas_mec = 0;
      if (f_mtr_torre_travado || f_mtr_pista_travado || f_mtr_leit_travado) {
        exibir_reinitiate();
        flag_botao_habilitado = false;
      } else {
        if (flag_botao_habilitado) {
          exibirStandBy();
        } else {
          exibir_waiting_connect();
        }
      }
      //tratamento do acionamento do botão
      if (flag_botao) {
        if (flag_botao_habilitado) {
          switch (modoLED) {
            case 1:  //auto
              Serial.print(botao_start);
              estado = 1;
              break;
            case 2:  //calibração
              break;
          }
          flag_autoteste = false;
        }
        flag_botao = false;
      }


      //tratamento de palavras na serial
      if (flag_p_iniciar_modo_zerar) {  //"#S1%C1011&"
        flag_p_iniciar_modo_zerar = false;
        estado = 1;
        flag_zerar = true;
      }

      if (flag_p_iniciar) {  //"#S1%C0001&"
        flag_p_iniciar = false;
        flag_autoteste = false;
        estado = 1;
      }

      if (flag_p_autoteste) {  //"#S1%C001&"//tratamento da tensão da pmt
        exibirAutoteste();
        flag_p_autoteste = false;
        dado_int = 0;           //zera a variável auxiliar no calculo da luz de referência
        luz_de_referencia = 0;  //zera variável luz de referência
        estado = 1;
        flag_autoteste = true;
      }

      if (flag_p_ler) {  //"#S1%C0111&" //palavra do modo manual, não mais suportado
        flag_p_ler = false;
        if (f_dos_mod_leit) {
          estado = 6;
        } else {
          estado = 1;
        }
      }

      if (flag_p_zerar) {  //"#S1%C1000&"
        flag_p_zerar = false;
        if (f_dos_mod_leit) {
          estado = 15;
        } else {
          estado = 1;
          flag_zerar = true;
        }
      }

      if (flag_p_dosimetro_ok) {  //"#S1%C1000&"
        flag_p_dosimetro_ok = false;
        if (f_dos_mod_leit) {
          estado = 19;
        } else {
          estado = 0;
        }
      }

      if (flag_p_dosimetro_n_ok) {  //"#S1%C1000&"
        flag_p_dosimetro_n_ok = false;
        if (f_dos_mod_leit) {
          estado = 21;
        } else {
          estado = 0;
        }
      }

      if (flag_p_SUDO_leitura) {
        flag_p_SUDO_leitura = false;
        //Serial.println("Comando sudo leitura recebido");
        flag_sudo = true;
        flag_autoteste = false;
        estado = 9;
      }

      if (flag_p_SUDO_zerar) {
        flag_p_SUDO_zerar = false;
        flag_sudo = true;
        flag_autoteste = false;
        estado = 17;
      }

      if (flag_p_SUDO_ligaLed) {
        flag_p_SUDO_ligaLed = false;
        flag_sudo = true;
        flag_autoteste = false;
        estado = 0;
        liga_led();
      }

      if (flag_p_parar == true) {
        flag_p_parar = false;
        flag_autoteste = false;
        estado = 0;
      }
      break;
    case 1:  //Acionar o periférico motor para posicinar leitora em origem
      f_p_sucesso_pos = false;
      f_p_falha_pos = false;
      if (flag_p_parar == true) {
        flag_p_parar = false;
        Serial.print(parado);
        f_mtr_torre_travado = false;
        f_mtr_pista_travado = false;
        f_mtr_leit_travado = false;
        estado = 0;
        break;
      }
      //se o sistema supervisório não interromper as atividades elas continuam
      Serial3.print(origem);  //requisita a posição origem ao periférico motor
      if (!flag_autoteste) {
        Serial.print(buscando_origem);  //informa ao supervisório buscando origem
      }
      estado = 2;
      if (flag_sudo) {
        f_mtr_torre_travado = false;
        f_mtr_pista_travado = false;
        f_mtr_leit_travado = false;
        estado = 0;
        flag_sudo = false;
      }
      //-------------------------
      time_out_mod_motor = millis();
      //-------------------------
      break;
    case 2:  //esperar confirmação de posicionamento do periferico motor
      if (f_p_falha_pos) {
        f_p_falha_pos = false;
        delay(1000);
        contador_falhas_mec++;
        if (contador_falhas_mec > 3) {
          if (!flag_autoteste) {
            Serial.print(travamento);
            exibirFalhaMecanica();
            estado = 0;
          } else {
            estado = 3;
          }
        } else {  // tenta de posicionar novamente
          estado = 1;
        }
      }
      if (f_p_sucesso_pos) {
        f_p_sucesso_pos = false;
        contador_falhas_mec = 0;
        estado = 3;

        if (flag_p_parar == true) {
          flag_p_parar = false;
          Serial.print(parado);
          estado = 0;
          f_mtr_torre_travado = false;
          f_mtr_pista_travado = false;
          f_mtr_leit_travado = false;
        }
      }
      //-----------------------------------------
      //time_out
      if (millis() - time_out_mod_motor > 20000) {
        estado = estado - 1;
      }
      //-----------------------------------------
      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }
      break;
    case 3:  //esperar confirmação de presença de dosímetro
      if (f_sem_dosimetro) {
        f_sem_dosimetro = false;
        if (!flag_autoteste) {
          Serial.print(fim_torre);
          exibir_leitura_completa();
          Serial.print(torre_vazia);
          f_mtr_torre_travado = false;
          f_mtr_pista_travado = false;
          f_mtr_leit_travado = false;
          estado = 0;
        } else {
          estado = 4;
        }
      }
      if (f_com_dosimetro) {
        f_com_dosimetro = false;
        estado = 4;
      }
      //-----------------------------------------
      //time_out
      if (millis() - time_out_mod_motor > 20000) {
        estado = estado - 1;
      }
      //-----------------------------------------
      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }
      break;
    case 4:  //requisitar que o periférico motor posicione o dosimetro fora da torre
      Serial3.print(dos_fora_torre);
      estado = 5;
      if (!flag_autoteste) {           //reportar ou não ao usuário o estado
        Serial.print(remo_dos_torre);  //informa ao supervisório removendo o dosímetro da torre
        exibir_numDosimetro();
      }
      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }
      //-------------------------
      time_out_mod_motor = millis();
      //-------------------------
      break;
    case 5:
      if (f_p_falha_pos) {
        f_p_falha_pos = false;
        delay(1000);
        contador_falhas_mec++;
        if (contador_falhas_mec > 3) {
          //abaixo o código específico
          if (!flag_autoteste) {
            Serial.print(travamento);
            exibirFalhaMecanica();
            estado = 0;
          } else {  //descarta o dosímetro para que ele não interfira na leitura da luz de led
            estado = 21;
          }
          //fim do código específico
        } else {  // tenta de posicionar novamente
          estado = 4;
        }
      }


      if (f_p_sucesso_pos) {
        f_p_sucesso_pos = false;
        contador_falhas_mec = 0;
        if (!flag_autoteste) {
          estado = 6;
          Serial.print(lendo_dos);  //lendo código de barras
        } else {
          estado = 21;
        }
      }
      //-----------------------------------------
      //time_out
      if (millis() - time_out_mod_motor > 20000) {
        estado = estado - 1;
      }
      //-----------------------------------------

      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }
      break;
    case 6:  //requisitar que o periférico motor ponha o dosímetro na posição HP10
      Serial3.print(HP10);
      estado = 7;

      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }

      //-------------------------
      time_out_mod_motor = millis();
      //-------------------------
      break;
    case 7:  //esperar confirmação de posicionamento HP10
      if (f_p_falha_pos) {
        f_p_falha_pos = false;
        delay(1000);
        contador_falhas_mec++;
        if (contador_falhas_mec > 3) {
          //abaixo o código específico
          if (!flag_autoteste) {
            Serial.print(travamento);
            exibirFalhaMecanica();
            estado = 0;
          } else {
            estado = 9;
          }
          //fim do código específico
        } else {  // tenta de posicionar novamente
          estado = 6;
        }
      }

      if (f_p_sucesso_pos) {
        contador_falhas_mec = 0;
        f_p_sucesso_pos = false;
        if (!flag_autoteste) {
          if (!flag_zerado_1_vez) {
            Serial.print(lendo_dose);  //fazendo leitura de dose
          } else {
            Serial.print(relendo);  //relendo
          }
          estado = 8;
          if (!flag_zerado_1_vez && flag_zerar) {
            //atual
            //Serial.print(fim_hp10);
            //delay(100);
            //Serial.print(fim_hp007);
            //delay(100);
            //Serial.print(fim_leit);
            f_dos_mod_leit = true;
            estado = 15;
          }
        } else {
          estado = 9;
        }
        time_out_supervisorio = millis();
      }
      //-----------------------------------------
      //time_out
      if (millis() - time_out_mod_motor > 20000) {
        estado = estado - 1;
      }
      //-----------------------------------------

      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }
      break;
    case 8:
      if (!f_dos_mod_leit) {
        if (flag_p_codigo_lido) {  //verifica se o código de barras  foi lido
          f_dos_mod_leit = true;
          flag_p_codigo_lido = false;
        } else if (flag_p_dosimetro_n_ok) {
          flag_p_dosimetro_n_ok = false;
          estado = 21;  //descarte não ok
        } else if (millis() - time_out_supervisorio > 3000) {
          Serial.print(time_out_super_p);
          time_out_supervisorio = millis();
          estado = 6;  //se não recebeu a confirmação do supervisório por mais de 1 min, e portanto deve repetir os passos anteriores
          contador_time_out_supervisorio++;
          if (contador_time_out_supervisorio > 3) {
            exibir_timeout();
            estado = 8;
          }
        }
      } else {  // esse else é diferente do if posterior. não é para juntar os dois códigos
        exibir_checking();
      }

      if (f_dos_mod_leit) {
        estado = 9;
      }

      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }
      break;
    case 9:  //começar leitura hp10
      contador_saturacao = 0;
      estado = 10;
      //----------
      flag_saturacao_pmt_p4 = false;  //reseta a flag de saturação
      flag_saturacao_pmt_p3 = false;  //reseta a flag de saturação
      //----------

      f_correcao_pmt = 1;        //reseta o fator de correção para 1
      iled_leitura = 0;          //zera variável de aquisição da corrente do led para o modo autoteste, uma linha de ódigo, não vale a pena fazer o teste de estar ou não no modo autoteste
      zeramento_ou_leitura = 1;  //define que o led a ser ligado é o de leitura
      aplicarParametros();
      delay(500);  //tempo necessário para ter certeza da estabilização da troca do zeramento para led de leitura
      time_out_mod_contador = millis();
      liga_led();
      //Serial2.print("start&");
      Serial1.print("start&");  //placa protótipo usa a serial 1 e não a serial 2
      //Serial.print("start&");
      break;
    case 10:  //Acionar o periférico contagem para fazer leitura hp10 ou luz de referência
      if (novoDado) {
        time_out_mod_contador = millis();
        if (canal < numeroCanais) {
          if (flag_autoteste) {  //Lê correte de estimulação e luz incidida na fotomultiplicadora
            dado_int = dado.toInt();
            luz_de_referencia += dado_int;
            iled_leitura += analogRead(A1);
          } else {
            iled_leitura = analogRead(A1);
            exibir_corrente_LED_leit(iled_leitura);
            int dens_pot_led_leitura = analogRead(A0)*4.88;
            exibir_densidade_pot_LED_leit(dens_pot_led_leitura);
            exibirDadosHP10(dado);  //reporta os dados ao supervisório
          }
          canal++;
        } else {
          canal = 0;
          desliga_led();
          if (!flag_autoteste) {
            Serial.print(fim_hp10);
          }
          Serial2.print("stop&");
          Serial1.print("stop&");  //placa protótiop usa a serial 1 para comunicar com o contador
          estado = 11;
          if (flag_sudo) {  //caso tenha sido feito um comando sudo para chegar aqui o mega volta para standby
            estado = 0;
            flag_sudo = false;
          }
        }
        dado = "";
        novoDado = false;
      } else if (millis() - time_out_mod_contador > 10000) {
        Serial2.print("stop&");
        Serial1.print("stop&");  //placa protótiop usa a serial 1 para comunicar com o contador
        time_out_mod_contador = millis();
        contador_time_out_mod_contador++;
        if (contador_time_out_mod_contador > 3) {
          exibir_timeout_contador();
        }
        delay(500);
        Serial2.print("start&");
        Serial1.print("start&");  //placa protótiop usa a serial 1 para comunicar com o contador
      }
      if (flag_p_SUDO_stop) {
        estado = 0;
        canal = 0;
        desliga_led();
        flag_sudo = false;
        Serial2.print("stop&");
        Serial1.print("stop&");  //placa protótiop usa a serial 1 para comunicar com o contador
      }
      break;
    case 11:  //Acionar o periférico motor para posicinar dosímetro para leitura HP007
      Serial3.print(HP07);
      estado = 12;
      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }

      //-------------------------
      time_out_mod_motor = millis();
      //-------------------------

      break;
    case 12:  //esperar confirmação de posicionamento HP007
      if (f_p_falha_pos) {
        f_p_falha_pos = false;
        delay(1000);
        contador_falhas_mec++;
        if (contador_falhas_mec > 3) {
          //abaixo o código específico
          if (!flag_autoteste) {
            Serial.print(travamento);
            exibirFalhaMecanica();
            estado = 0;
          } else {
            estado = 15;
          }
          //fim do código específico
        } else {  // tenta de posicionar novamente
          estado = 11;
        }
      }


      if (f_p_sucesso_pos) {
        contador_falhas_mec = 0;
        f_p_sucesso_pos = false;
        if (!flag_autoteste) {
          estado = 13;
        } else {
          estado = 15;
        }
      }

      //-----------------------------------------
      //time_out
      if (millis() - time_out_mod_motor > 10000) {
        estado = estado - 1;
      }
      //-----------------------------------------

      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }
      break;
    case 13:  //acionar o periférico contagem
      contador_saturacao = 0;
      //----------
      flag_saturacao_pmt_p4 = false;  //reseta a flag de saturação
      flag_saturacao_pmt_p3 = false;  //reseta a flag de saturação
      //----------

      f_correcao_pmt = 1;        //reseta o fator de correção para 1
      zeramento_ou_leitura = 1;  //define que o led a ser ligado é o de leitura
      aplicarParametros();       //necessário no caso em que os parâmetros mudem entre a leitura hp10 e hp07
      delay(2000);
      liga_led();
      Serial2.print("start&");
      Serial1.print("start&");  //placa protótipo usa a serial 1
      estado = 14;

      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }
      break;
    case 14:  //Adiquirir contagem e enviar ao supervisório HP007
      if (novoDado) {
        if (canal < numeroCanais) {
          exibirDadosHP07(dado);
          canal++;
        } else {
          //estado = 15;
          canal = 0;
          desliga_led();
          if (!flag_autoteste) {
            Serial.print(fim_hp007);
          }
          Serial2.print("stop&");
          Serial1.print("stop&");  //placa protótipo
          if (!flag_autoteste) {
            if (flag_zerado_1_vez) {
              flag_p_zerar = false;
              Serial.print(fim_releit);  //ciclo completo, fim da releitura
              time_out_supervisorio = millis();
            } else {
              Serial.print(fim_leit);  //fim da leitura
              time_out_supervisorio = millis();
            }
          }
          estado = 18;
        }
        dado = "";
        novoDado = false;
      }

      if (flag_p_SUDO_stop) {
        estado = 0;
        canal = 0;
        desliga_led();
        flag_sudo = false;
        Serial2.print("stop&");
        Serial1.print("stop&");  //placa protótipo
      }
      break;
    case 15:  //Acionar o periférico motor para posicionar zeramento
      Serial3.print(zeramento);
      estado = 16;

      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }
      //-------------------------
      time_out_mod_motor = millis();
      //-------------------------
      break;
    case 16:  //esperar confirmação de posicionamento zeramento
      if (f_p_falha_pos) {
        f_p_falha_pos = false;
        delay(1000);
        contador_falhas_mec++;
        if (contador_falhas_mec > 3) {
          //abaixo o código específico
          if (!flag_autoteste) {
            Serial.print(travamento);
            exibirFalhaMecanica();
            estado = 0;
          } else {
            estado = 17;
          }
          //fim do código específico
        } else {  // tenta de posicionar novamente
          estado = 15;
        }
      }

      if (f_p_sucesso_pos) {
        contador_falhas_mec = 0;
        f_p_sucesso_pos = false;
        if (!flag_autoteste) {
          Serial.print(zerando);  //apagando dosimetro, zerando
        }
        estado = 17;
      }

      //-----------------------------------------
      //time_out
      if (millis() - time_out_mod_motor > 20000) {
        estado = estado - 1;
      }
      //-----------------------------------------
      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }
      break;
    case 17:                     //Acionar os leds de zeramento (estado zeramento)
      zeramento_ou_leitura = 0;  //define que o led a ser ligado é o de zeramento
      aplicarParametros();

      liga_led();
      if (flag_autoteste) {  //não exibe dados
        n_leit_i_zeramento = tempo_zeramento / 100;
        for (int i = 0; i < n_leit_i_zeramento; i++) {
          iled_zeramento += analogRead(A1);
          delay(100);
        }
        desliga_led();
        estado = 19;
      } else {
        exibir_zerando();
        delay(tempo_zeramento / 2);
        delay(tempo_zeramento / 2);
        desliga_led();
        exibir_zeramentofinalizado();
        estado = 6;
        flag_zerado_1_vez = true;
      }

      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }
      break;
    case 18:  //esperar direcionamento do supervisório para descarte ou zerar
      if (millis() - time_out_supervisorio > 5000) {
        Serial.print(time_out_super_p);
        time_out_supervisorio = millis();

        if (flag_zerado_1_vez) {
          Serial.print(fim_releit);  //ciclo completo, fim da releitura
        } else {
          Serial.print(fim_leit);  //fim da leitura
        }

        contador_time_out_supervisorio++;
        if (contador_time_out_supervisorio > 3) {
          exibir_timeout();
          delay(2000);
        }
      }


      //a sequência  desses ifs é importante pois ela decide qual dos comandos é prioridade, o ultimo é a maior priuoridade quando mais de um é enviado ao mesmo tempo
      if (flag_p_zerar) {
        flag_p_zerar = false;
        estado = 15;
      }

      if (flag_p_dosimetro_ok) {
        flag_p_dosimetro_ok = false;
        flag_zerado_1_vez = false;
        estado = 19;
      }

      if (flag_p_dosimetro_n_ok) {
        flag_p_dosimetro_n_ok = false;
        flag_zerado_1_vez = false;
        estado = 21;
      }

      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }
      break;
    case 19:  //Acionar o periférico motor para Posicionar em descarte ok
      if (!flag_autoteste) {
        exibir_Dosimetro_OK();
        Serial.print(ejetando);  //ejetanto
      }
      Serial3.print(descarte_ok);
      estado = 20;
      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }
      //-------------------------
      time_out_mod_motor = millis();
      //-------------------------
      break;
    case 20:  //esperar confirmação de posicionamento de descarte ok
      if (f_p_falha_pos) {
        f_p_falha_pos = false;
        delay(1000);
        contador_falhas_mec++;
        if (contador_falhas_mec > 3) {
          //abaixo o código específico
          if (!flag_autoteste) {
            //Serial.print(travamento);
            exibirFalhaMecanica();
            estado = 21;
          } else {
            estado = 23;
          }
          //fim do código específico
        } else {  // tenta de posicionar novamente
          estado = 19;
        }
      }

      if (f_p_sucesso_pos) {
        contador_falhas_mec = 0;
        f_p_sucesso_pos = false;
        f_dos_mod_leit = false;
        if (!flag_autoteste) {
          Serial.print(dos_descartado);  //finalizado 1 dosimetro
        }
        if (flag_autoteste) {
          flag_autoteste_fim = true;
          estado = 21;
        } else {  //automático
          estado = 1;
          f_mtr_torre_travado = false;
          f_mtr_pista_travado = false;
          f_mtr_leit_travado = false;
        }
      }

      //-----------------------------------------
      //time_out
      if (millis() - time_out_mod_motor > 20000) {
        estado = estado - 1;
      }
      //-----------------------------------------

      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }
      break;
    case 21:  //Acionar o periférico motor para Posicionar em descarte não ok
      if (!flag_autoteste) {
        Serial.print(ejetando);  //ejetanto
        exibir_Dosimetro_NOK();
      }
      Serial3.print(descarte_n_ok);
      estado = 22;
      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }
      //-------------------------
      time_out_mod_motor = millis();
      //-------------------------
      break;
    case 22:  //esperar confirmação de posicionamento de descarte não ok
      if (f_p_falha_pos) {
        f_p_falha_pos = false;
        delay(1000);
        contador_falhas_mec++;
        if (contador_falhas_mec > 3) {
          //abaixo o código específico
          if (!flag_autoteste) {
            Serial3.print(descrt_n_ok_desacoplado);
            if (contador_falhas_mec > 6) {
              Serial.print(travamento);
              exibirFalhaMecanica();
              estado = 0;
            }
          } else {
            if (flag_autoteste_fim) {
              estado = 23;
              flag_autoteste_fim = false;
            } else {
              estado = 6;
            }
          }
          //fim do código específico
        } else {  // tenta de posicionar novamente
          estado = 19;
        }
      }

      if (f_p_sucesso_pos) {
        contador_falhas_mec = 0;
        f_p_sucesso_pos = false;
        f_dos_mod_leit = false;
        if (!flag_autoteste) {
          Serial.print(dos_descartado);  //finalização de leitura
        }
        if (flag_autoteste) {
          if (flag_autoteste_fim) {
            estado = 23;
            flag_autoteste_fim = false;
          } else {
            estado = 6;
          }
        } else {  //modo automatico de funcionamento faz com que o sistema reinicie o processo
          estado = 1;
          f_mtr_torre_travado = false;
          f_mtr_pista_travado = false;
          f_mtr_leit_travado = false;
        }
      }
      //-----------------------------------------
      //time_out
      if (millis() - time_out_mod_motor > 20000) {
        estado = estado - 1;
      }
      //-----------------------------------------

      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }
      break;
    case 23:  //Envia resultados para o computado do autoteste corrente de motor e escreve para o pc
      exibir_luz_referencia(luz_de_referencia);
      iled_leitura = iled_leitura / numeroCanais;
      exibir_corrente_LED_leit(iled_leitura);
      iled_zeramento = iled_zeramento / n_leit_i_zeramento;
      exibir_corrente_LED_zeramento(iled_zeramento);
      exibir_VPMT();
      if (f_mtr_torre_travado) {
        Serial.print(mtr_torre_travado);
      } else if (f_mtr_pista_travado) {
        Serial.print(mtr_pista_travado);
      } else if (f_mtr_leit_travado) {
        Serial.print(mtr_leit_travado);
      } else {
        Serial.print(motores_bem);
      }
      f_mtr_torre_travado = false;
      f_mtr_leit_travado = false;
      f_mtr_leit_travado = false;
      iled_leitura = 0;
      iled_zeramento = 0;
      flag_autoteste = false;
      estado = 0;
      if (flag_sudo) {
        estado = 0;
        flag_sudo = false;
      }
      break;
  }
}
//==================================================================
void liga_led() {
  digitalWrite(liga_desliga_led, HIGH);  // liga led
}
//==================================================================
void desliga_led() {
  digitalWrite(liga_desliga_led, LOW);  // liga led
}
//==================================================================
void exibirDadosHP10(String dado) {
  // dados: #S1%Dxxxxxxx&
  // informacao: #S1%IsatLeit&, #S1%IstandBy&
  long int contador = dado.toInt();
  //----------------
  //flag_saturacao_pmt_g3 = true; //teste para mudar do ganho 3 diretamente para o ganho 1
  //----------------

  contador = contador * f_correcao_pmt;
  dado = (String)contador;

  //lcd.setCursor(1, 0);
  //lcd.print("--");
  //lcd.print(dado);
  //lcd.print("--");
  //------------------------------------------------------------------
  if (contador >= (2000000 * f_correcao_pmt) && !flag_saturacao_pmt_p3) {
    if (potenciaLEDLAux == "4") {
      if (!flag_saturacao_pmt_p4) {
        desliga_led();
        Serial2.print("stop&");
        Serial1.print("stop&");  //placa protótipo
        flag_saturacao_pmt_p4 = true;
        //ganhoPmt = calcula_ganho_pmt(ganhoPmtAux - 1);
        //definir_ganho_pmt(ganhoPmt);
        f_correcao_pmt = 10;
        seleciona_potencia_led(3);
        delay(5000);
        time_out_mod_contador = millis();
        Serial2.print("start&");
        Serial1.print("start&");  //placa protótipo
        liga_led();
      } else if (!flag_saturacao_pmt_p3) {
        desliga_led();
        Serial2.print("stop&");
        Serial1.print("stop&");  //placa protótipo
        flag_saturacao_pmt_p3 = true;
        //ganhoPmt = calcula_ganho_pmt(ganhoPmtAux - 1);
        //definir_ganho_pmt(ganhoPmt);
        f_correcao_pmt = 100;
        seleciona_potencia_led(2);
        delay(5000);
        time_out_mod_contador = millis();
        Serial2.print("start&");
        Serial1.print("start&");  //placa protótipo
        liga_led();
      }
    }
  }
  //-------------------------------------------------------------------
  if (contador >= 0 && contador <= 9) {
    Serial.print("#L1%A00000000" + dado + "&");
  } else if (contador >= 10 && contador <= 99) {
    Serial.print("#L1%A0000000" + dado + "&");
  } else if (contador >= 100 && contador <= 999) {
    Serial.print("#L1%A000000" + dado + "&");
  } else if (contador >= 1000 && contador <= 9999) {
    Serial.print("#L1%A00000" + dado + "&");
  } else if (contador >= 10000 && contador <= 99999) {
    Serial.print("#L1%A0000" + dado + "&");
  } else if (contador >= 100000 && contador <= 999999) {
    Serial.print("#L1%A000" + dado + "&");
  } else if (contador >= 1000000 && contador < 9999999) {
    Serial.print("#L1%A00" + dado + "&");
  } else if (contador >= 10000000 && contador < 99999999) {
    Serial.print("#L1%A0" + dado + "&");
  } else if (contador >= 100000000 && contador < 999999999) {
    Serial.print("#L1%A" + dado + "&");
  }

  //------------------------------------------------------------------
}
//==================================================================
void exibirDadosHP07(String dado) {
  // dados: #S1%Dxxxxxxx&
  // informacao: #S1%IsatLeit&, #S1%IstandBy&
  long int contador = dado.toInt();
  //----------------
  contador = (long)contador * f_correcao_pmt;
  dado = (String)contador;
  //lcd.setCursor(1, 0);
  //lcd.print("--");
  //lcd.print(dado);
  //lcd.print("--");
  //------------------------------------------------------------------
  if (contador >= (2000000 * f_correcao_pmt) && !flag_saturacao_pmt_p3) {
    if (potenciaLEDLAux == "4") {
      if (!flag_saturacao_pmt_p4) {
        desliga_led();
        Serial2.print("stop&");
        Serial1.print("stop&");  //placa protótipo
        flag_saturacao_pmt_p4 = true;
        //ganhoPmt = calcula_ganho_pmt(ganhoPmtAux - 1);
        //definir_ganho_pmt(ganhoPmt);
        f_correcao_pmt = 10;
        seleciona_potencia_led(3);
        delay(5000);
        time_out_mod_contador = millis();
        Serial2.print("start&");
        Serial1.print("start&");  //placa protótipo
        liga_led();
      } else if (!flag_saturacao_pmt_p3) {
        desliga_led();
        Serial2.print("stop&");
        Serial1.print("stop&");  //placa protótipo
        flag_saturacao_pmt_p3 = true;
        //ganhoPmt = calcula_ganho_pmt(ganhoPmtAux - 1);
        //definir_ganho_pmt(ganhoPmt);
        f_correcao_pmt = 100;
        seleciona_potencia_led(2);
        delay(5000);
        time_out_mod_contador = millis();
        Serial2.print("start&");
        Serial1.print("start&");  //placa protótipo
        liga_led();
      }
    }
  }
  //-------------------------------------------------------------------
  if (contador >= 0 && contador <= 9) {
    Serial.print("#L1%B00000000" + dado + "&");
  } else if (contador >= 10 && contador <= 99) {
    Serial.print("#L1%B0000000" + dado + "&");
  } else if (contador >= 100 && contador <= 999) {
    Serial.print("#L1%B000000" + dado + "&");
  } else if (contador >= 1000 && contador <= 9999) {
    Serial.print("#L1%B00000" + dado + "&");
  } else if (contador >= 10000 && contador <= 99999) {
    Serial.print("#L1%B0000" + dado + "&");
  } else if (contador >= 100000 && contador <= 999999) {
    Serial.print("#L1%B000" + dado + "&");
  } else if (contador >= 1000000 && contador < 9999999) {
    Serial.print("#L1%B00" + dado + "&");
  } else if (contador >= 10000000 && contador < 99999999) {
    Serial.print("#L1%B0" + dado + "&");
  } else if (contador >= 100000000 && contador < 999999999) {
    Serial.print("#L1%B" + dado + "&");
  }

  //------------------------------------------------------------------
}
//==================================================================
void exibir_VPMT() {
  // tensão pmt: #L1%Vxxxxxx&
  int analogPin = A0;
  int VPMT = analogRead(analogPin);
  VPMT = VPMT * 4.88;  //(mV)
  String vpmt_string = (String)VPMT;
  if (VPMT >= 0 && VPMT <= 9) {
    Serial.print("#L1%V000000" + vpmt_string + "&");
  } else if (VPMT >= 10 && VPMT <= 99) {
    Serial.print("#L1%V00000" + vpmt_string + "&");
  } else if (VPMT >= 100 && VPMT <= 999) {
    Serial.print("#L1%V0000" + vpmt_string + "&");
  } else if (VPMT >= 1000 && VPMT <= 9999) {
    Serial.print("#L1%V000" + vpmt_string + "&");
  } else if (VPMT >= 10000 && VPMT <= 99999) {
    Serial.print("#L1%V00" + vpmt_string + "&");
  } else if (VPMT >= 100000 && VPMT <= 999999) {
    Serial.print("#L1%V0" + vpmt_string + "&");
  } else if (VPMT >= 1000000 && VPMT < 1900000) {
    Serial.print("#L1%V" + vpmt_string + "&");
  } else if (VPMT >= 1900000) {
    Serial.print("#L1%VsatLeit&");  // verificar valor de sat PMT
  }
}
//==================================================================
void exibir_corrente_LED_leit(int iled) {
  // corrente led: #L1%Exxxxxx&
  iled = iled * 8 + 18;  //mA
  String iled_string = (String)iled;
  if (iled >= 0 && iled <= 9) {
    Serial.print("#L1%E000000" + iled_string + "&");
  } else if (iled >= 10 && iled <= 99) {
    Serial.print("#L1%E00000" + iled_string + "&");
  } else if (iled >= 100 && iled <= 999) {
    Serial.print("#L1%E0000" + iled_string + "&");
  } else if (iled >= 1000 && iled <= 9999) {
    Serial.print("#L1%E000" + iled_string + "&");
  } else if (iled >= 10000 && iled <= 99999) {
    Serial.print("#L1%E00" + iled_string + "&");
  } else if (iled >= 100000 && iled <= 999999) {
    Serial.print("#L1%E0" + iled_string + "&");
  } else if (iled >= 1000000 && iled < 1900000) {
    Serial.print("#L1%E" + iled_string + "&");
  } else if (iled >= 1900000) {
    Serial.print("#L1%EsatLeit&");  // verificar valor de sat corr
  }
}
//==================================================================
void exibir_densidade_pot_LED_leit(int dens_pot_led) {
  // densidade_led: #L1%Dxxxxxx&
  //mV
  String iled_string = (String)dens_pot_led;
  if (dens_pot_led >= 0 && dens_pot_led <= 9) {
    Serial.print("#L1%D000000" + iled_string + "&");
  } else if (dens_pot_led >= 10 && dens_pot_led <= 99) {
    Serial.print("#L1%D00000" + iled_string + "&");
  } else if (dens_pot_led >= 100 && dens_pot_led <= 999) {
    Serial.print("#L1%D0000" + iled_string + "&");
  } else if (dens_pot_led >= 1000 && dens_pot_led <= 9999) {
    Serial.print("#L1%D000" + iled_string + "&");
  } else if (dens_pot_led >= 10000 && dens_pot_led <= 99999) {
    Serial.print("#L1%D00" + iled_string + "&");
  } else if (dens_pot_led >= 100000 && dens_pot_led <= 999999) {
    Serial.print("#L1%D0" + iled_string + "&");
  } else if (dens_pot_led >= 1000000 && dens_pot_led < 1900000) {
    Serial.print("#L1%E" + iled_string + "&");
  } else if (dens_pot_led >= 1900000) {
    Serial.print("#L1%DsatLeit&");  // verificar valor de sat dens
  }
}
//==================================================================
void exibir_corrente_LED_zeramento(int iled) {
  // corrente led: #L1%Exxxxxx&
  iled = iled * 8 + 18;  //mA
  String iled_string = (String)iled;
  if (iled >= 0 && iled <= 9) {
    Serial.print("#L1%F000000" + iled_string + "&");
  } else if (iled >= 10 && iled <= 99) {
    Serial.print("#L1%F00000" + iled_string + "&");
  } else if (iled >= 100 && iled <= 999) {
    Serial.print("#L1%F0000" + iled_string + "&");
  } else if (iled >= 1000 && iled <= 9999) {
    Serial.print("#L1%F000" + iled_string + "&");
  } else if (iled >= 10000 && iled <= 99999) {
    Serial.print("#L1%F00" + iled_string + "&");
  } else if (iled >= 100000 && iled <= 999999) {
    Serial.print("#L1%F0" + iled_string + "&");
  } else if (iled >= 1000000 && iled < 1900000) {
    Serial.print("#L1%F" + iled_string + "&");
  } else if (iled >= 1900000) {
    Serial.print("#L1%FsatLeit&");  // verificar valor de sat PMT
  }
}
//==================================================================
void exibir_luz_referencia(int luz_referencia) {
  // luz de referência: #L1%Ixxxxxx&
  String luz_referencia_string = (String)luz_referencia;
  if (luz_referencia >= 0 && luz_referencia <= 9) {
    Serial.print("#L1%L000000" + luz_referencia_string + "&");
  } else if (luz_referencia >= 10 && luz_referencia <= 99) {
    Serial.print("#L1%L00000" + luz_referencia_string + "&");
  } else if (luz_referencia >= 100 && luz_referencia <= 999) {
    Serial.print("#L1%L0000" + luz_referencia_string + "&");
  } else if (luz_referencia >= 1000 && luz_referencia <= 9999) {
    Serial.print("#L1%L000" + luz_referencia_string + "&");
  } else if (luz_referencia >= 10000 && luz_referencia <= 99999) {
    Serial.print("#L1%L00" + luz_referencia_string + "&");
  } else if (luz_referencia >= 100000 && luz_referencia <= 999999) {
    Serial.print("#L1%L0" + luz_referencia_string + "&");
  } else if (luz_referencia >= 1000000 && luz_referencia < 1900000) {
    Serial.print("#L1%L" + luz_referencia_string + "&");
  } else if (luz_referencia >= 1900000) {
    Serial.print("#L1%LsatLeit&");  // verificar valor de sat PMT
  }
}
//==================================================================
void interrupcao_2() {
  if (millis() - debounce_bt_start_stop > 200) {
    debounce_bt_start_stop = millis();
    flag_botao = true;
  }
}
//=========================================
//****************************************************
// definir parametros
//****************************************************
void definirParametros() {
  //   0 1 2 3 4 5 6 7 8 9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24
  //  "# S 1 % M X G X L X  X   X   X   X   P   X   Z   X   X   X   X   X   Q   X   &" -> definicao de (L) tempo de leitura, (G) ganho pmt, (P) potencia led, (M) modo led, (Z) tempo de zeramento, (Q) potência de zeramento
  //  #S1%MXGXLXXXXXPXZXXXXXQX&
  //  #S1%M3G3L03000P4Z05000Q2&
  tempoLeituraAux = palavra.substring(9, 14);  //L
  tempoLeitura = tempoLeituraAux.toInt();      // tempo de leitura
  ganhoPmtAux = palavra[7];                    //G
  //---------------------
  ganhoPmt = calcula_ganho_pmt(ganhoPmtAux);
  //---------------------
  potenciaLEDLAux = palavra.substring(15, 16);  //P
  potenciaLEDL = potenciaLEDLAux.toInt();       // intensidade de luz LED na leitura
  //implementando potência led específica para o zeramento
  potenciaLEDZAux = palavra.substring(23, 24);  //Q
  potenciaLEDZ = potenciaLEDZAux.toInt();       // intensidade de luz LED no zeramento

  modoLEDAux = palavra.substring(5, 6);      //M
  modoLED = modoLEDAux.toInt();              // modo de de funcionamento
  zeramentoAux = palavra.substring(17, 22);  //Z
  tempo_zeramento = zeramentoAux.toInt();    // tempo de zeramento
  numeroCanais = tempoLeitura / taxaAquisicao;
  // paramentros (eco serial...)

  //    Serial.println(String(tempoLeitura));     // tempo de leitura
  //    Serial.println(String(ganhoPmt));         // tipo de led (azul ou infravermelho)
  //    Serial.println(String(potenciaLEDL));     // pontecia led (25(1), 50(2), 75(3) e 100(4))%
  //    Serial.println(String(potenciaLEDZ));     // pontecia led (25(1), 50(2), 75(3) e 100(4))%
  //    Serial.println(String(modoLED));
  //    Serial.println(String(tempo_zeramento));
  //    Serial.println(String(numeroCanais));     // numero de amostragens por 0,1s
}

//==================================================================
int calcula_ganho_pmt(char ganho) {
  //    Serial.println();
  //    Serial.print("Ganho: ");
  //    Serial.println(ganho);
  switch (ganho) {  // ganho da pmt
    case '1':
      return 40;  //43 resultou em um ganho de 300 //50 resultou em multiplicador de 1224 vezes tudo isso a 10mSV
      break;
    case '2':
      return 30;  //29 resultou em um fator de 10 vezes //31 resultou em um fator de 15 vezes  //34 resultou em multiplicador de 22 vezes      // 27 resultou em multiplicador x5 tudo isso a 10mSV
      break;
    case '3':
      return 15;  //15 : 0,99830V //14 : 1,00237V
      break;
    case '0':
      return 50;
      break;
  }
}
//==================================================================
void aplicarParametros() {
  int pLED = 0;
  if (zeramento_ou_leitura == 1) {   // caso de leitura
    digitalWrite(selecao_led, LOW);  // led de leitura
    pLED = potenciaLEDL;
  } else {                            // caso de zeramento
    digitalWrite(selecao_led, HIGH);  // led de zeramento
    pLED = potenciaLEDZ;
  }
  // selecao potencia do led (25%, 50%, 75% e 100%)
  seleciona_potencia_led(pLED);

  ganhoPmt = calcula_ganho_pmt(ganhoPmtAux);
  definir_ganho_pmt(ganhoPmt);  //define o ganho da pmt
}
//==================================================================
void seleciona_potencia_led(int pLED) {
  if (pLED == 1) {            // corrente (10%)
    digitalWrite(Aled, LOW);  // A = 0
    digitalWrite(Bled, LOW);  // B = 0
  }
  if (pLED == 2) {             // corrente (50%)
    digitalWrite(Aled, HIGH);  // A = 0
    digitalWrite(Bled, LOW);   // B = 1
  }
  if (pLED == 3) {             // corrente (75%)
    digitalWrite(Aled, LOW);   // A = 0
    digitalWrite(Bled, HIGH);  // B = 1
  }
  if (pLED == 4) {             // corrente (100%)
    digitalWrite(Aled, HIGH);  // A = 1
    digitalWrite(Bled, HIGH);  // B = 1
  }
}
//==================================================================
void definir_ganho_pmt(int val) {
  SPI.begin();  // Set pins as outputs for SPI hardware.
  //  // take the CS pin low to select the chip:
  digitalWrite(CS, LOW);
  //  send in the address and value via SPI:
  SPI.transfer(B00010001);
  // write out the value
  SPI.transfer(val);
  // take the CS pin high to de-select the chip:
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
  //Serial.println("enviou o comando para o potdig");
}
//==================================================================
void trata_palavra_supervisorio() {
  if (novaPalavra) {  //tratamento da palavra vinda do sistema supervisório
    //Serial.println(palavra);
    if (palavra.substring(4, 5) == "M") {
      definirParametros();
      aplicarParametros();
    }
    if (palavra == iniciar) {
      flag_p_iniciar = true;
    }
    if (palavra == parar) {
      flag_p_parar = true;
    }
    if (palavra == autoteste) {
      flag_p_autoteste = true;
    }
    if (palavra == dosimetro_ok) {
      flag_p_dosimetro_ok = true;
    }
    if (palavra == dosimetro_n_ok) {
      flag_p_dosimetro_n_ok = true;
    }
    if (palavra == codigo_lido) {
      flag_p_codigo_lido = true;
    }
    if (palavra == ler) {  //feito
      flag_p_ler = true;
    }
    if (palavra == zerar) {
      flag_p_zerar = true;
    }
    if (palavra == habilitar_botao) {
      flag_botao_habilitado = true;
    }
    if (palavra == desabilitar_botao) {
      flag_botao_habilitado = false;
    }
    if (palavra == iniciar_modo_zerar) {
      flag_p_iniciar_modo_zerar = true;
    }
    //comandos sudo
    if (palavra == SUDO_leitura) {
      flag_p_SUDO_leitura = true;
    }
    if (palavra == SUDO_stop) {
      flag_p_SUDO_stop = true;
    }
    if (palavra == SUDO_zerar) {  //feito
      flag_p_SUDO_zerar = true;
    }
    if (palavra == SUDO_ligaLed) {
      flag_p_SUDO_ligaLed = true;
    }

    palavra = "";
    novaPalavra = false;
  }
}
//==================================================================
void trata_botao() {
  if (flag_botao) {
    if (estado != 0) {
      flag_botao = false;
      flag_p_parar = true;
      Serial.print(botao_stop);
    }
  }
}
//==================================================================
void trata_palavra_motor() {
  if (nova_palavra_3) {
    //Serial.println("");
    //Serial.println(palavra_3);
    if (palavra_3.equals(falha_motor_torre)) {
      f_mtr_torre_travado = true;
    }
    if (palavra_3.equals(falha_motor_pista)) {
      f_mtr_pista_travado = true;
    }
    if (palavra_3.equals(falha_motor_leit)) {
      f_mtr_leit_travado = true;
    }
    if (palavra_3.equals(falha_pos)) {
      f_p_falha_pos = true;
    }
    if (palavra_3.equals(sucesso_pos)) {
      f_p_sucesso_pos = true;
    }
    if (palavra_3.equals(sem_dosimetro)) {
      f_sem_dosimetro = true;
    }
    if (palavra_3.equals(com_dosimetro)) {
      f_com_dosimetro = true;
    }
    nova_palavra_3 = false;
    palavra_3 = "";
  }
}
// FIM DO CÓDIGO
//*********************************************************************
