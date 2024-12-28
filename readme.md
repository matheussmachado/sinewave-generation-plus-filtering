# Sine Wave Generation + Filtering (Analog & Real-Time IIR Digital Filter)

## Sobre
Projeto de Processamento Digital de Sinal (DSP). Foi utilizado um microcontrolador para: gerar uma senoide de frequência variável (a cada 4 períodos) e aquisição desse sinal gerado para processamento de filtro digital (Butterworth - ordem 2) em *real time*. Além disso, o mesmo sinal gerado foi inserido em um filtro analógico correspondente ao filtro digital, e tendo seu sinal aquisitado para fins de comparação.

### Features
- STM32F407GT6, programado em C via HAL na STM32CubeIDE
- DAC + DMA, para geração de sinal senoidal.
- ADC + DMA, para aquisição de sinal senoidal gerado e aquisição saída de filtro analógico.
- ARM CMSIS DSP Library v4, para cálculo otimizado de filtro IIR em tempo real.
- Scripts MATLAB/Octave, para design e validação de filtro digital.

#### Extra:
- Circuito de filtro ativo tipo Butterworth de segunda ordem
- MCUViewer, para visualização em tempo real e coleta de dados do sistema.


## Detalhes

### Overview do projeto

O projeto foi concebido da forma como ilustra a figura a seguir:


![diagrama](https://github.com/matheussmachado/sinewave-generation-plus-filtering/blob/main/diagrama.png)

Como mencionado anteriormente, o sinal da senoide gerada via DAC do MCU, bem como o sinal da saída do circuito do filtro ativo, foram aquisitados via ADC.

### Design do filtro

Os coeficientes do filtro Butterworth de segunda ordem, frequência de corte em 50 Hz e frequência de amostragem de 1 kHz foram obtidos através do scripts MATLAB/Octave presente no arquivo **cad/filter-design.m**. Os parâmetros, já formatados para as funções do *CMSIS Biquad Cascade IIR Filters Using Direct Form I Structure* foram:

```
CMSIS IIR FILTER COEFFICIENTS:
0.020083, 0.040167, 0.020083, 1.561018, -0.641352
```

Neste arquivo também foi projetado o filtro analógico, com base em componentes eletrônicos disponíveis:

![schematic](https://github.com/matheussmachado/sinewave-generation-plus-filtering/blob/main/schematic.png)

O desempenho de ambos os filtros foram simulados com sinal a ser reproduzido no microcontrolador. Os resultados das simulações podem ser visualizados a seguir:

![performance](https://github.com/matheussmachado/sinewave-generation-plus-filtering/blob/main/performance.png)

![bode](https://github.com/matheussmachado/sinewave-generation-plus-filtering/blob/main/bode.png)

### Montagem do sistema

* Circuito do filtro ativo

![cct_filtro_montado](https://github.com/matheussmachado/sinewave-generation-plus-filtering/blob/main/cct_filtro_montado.png)

* STM32 MCU + Filtro

![cct_projeto_montado](https://github.com/matheussmachado/sinewave-generation-plus-filtering/blob/main/cct_projeto_montado.png)

* Pinout utilizado

![stm32-pinout](https://github.com/matheussmachado/sinewave-generation-plus-filtering/blob/main/stm32-pinout.png)

Na imagem anterior, atentar para o pino *ADC3_IN1* usado para aquisitar o sinal do filtro analógico através da variável *adc_dma[0]* no firmware do microcontrolador. Além disso, o pino *ADC3_IN2* foi usado para aquisitar o sinal gerado no pino *DAC_OUT1* e tratado no firmware pela variável *adc_dma[1]*.

### Instrução de setup
- Foi utilizado o STM32CubeIDE v1.17.0
- Para conseguir compilar e utilizar a biblioteca *arm_math.h* é necessário adicionar essa *lib* no caminho de arquivos a serem compilados, através da STM32CubeIDE

## Resultados

Nas imagens a seguir, é possível visualizar projeto em funcionamento, que é a geração de sinal senoidal e filtragem deste sinal. Nesta mesma imagem, é possível verificar que foram monitoradas, através do MCUViewer, as variáveis:

- *adc_dma[0]*, saída do filtro analógico
- *adc_dma[1]*, saída do sinal gerado via DAC.
- *x[0]*, entrada do filtro digital (cópia de *adc_dma[1]*)
- *y[0]*, saída do filtro digital


![viewer](https://github.com/matheussmachado/sinewave-generation-plus-filtering/blob/main/viewer.png)

![viewer-zoom-out](https://github.com/matheussmachado/sinewave-generation-plus-filtering/blob/main/viewer-zoom-out.png)

Por fim, um vídeo demonstrando o projeto rodando em tempo real:

https://github.com/user-attachments/assets/e65a8332-0005-4723-843c-bf607c883942

 

