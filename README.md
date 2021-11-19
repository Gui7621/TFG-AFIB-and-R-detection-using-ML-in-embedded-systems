Os dados do projeto compartilhado nesse repositório foram divididos em determinadas pastas que envolvem diferentes etapas realizadas no TFG.

A primeira (Algoritmo de detecção de picos R) possui o arquivo em Python do algoritmo construído junto com os testes feitos para atestarem a sua funcionalidade.

Já a segunda pasta (Arquivo do esquemático no Fritzing) tem o arquivo para ser editado no Fritzing com o esquemático na protoboard do protótipo montado. 

A pasta "Arquivos do Jupyter Notebook" contém diversos arquivos em Python que mostram todo o desenvolvimento do projeto de aprendizado de máquina, desde o trabalho com a base de dados incluindo a filtragem e normalização dos registros até a construção e teste do modelo neural.

A pasta "Classificações do modelo para dados de ECG amostrados no Boneco Simulador SimMan" possui os arquivos em Excel contendo os resultados das classificações do modelo para os exames realizados no SimMan.

Já a pasta "Dados de ECG amostrados no Boneco Simulador SimMan" tem alguns arquivos em Excel que contém registros de ECG de RS e FA coletados a partir do SimMan, onde alguns foram utilizados para teste do modelo nos arquivos do Jupyter Notebook.

A sexta pasta (Link do projeto com o modelo final no Edge Impulse) contém um arquivo em txt com o link para o projeto público no Edge Impulse que possui o modelo neural utilizado para fazer a inferência na ESP32. Lá é possível também verificar os resultados de treinamento e teste do modelo, assim como os dados de entrada deste, que são justamente os batimentos cardíacos individuais.

Por fim, a pasta "Programa em C++" possui o arquivo .ino do programa final utilizado para fazer a inferência do modelo em C++ na ESP32, incluindo todas as etapas de amostragem do sinal cardíaco, tratamento dos dados, obtenção dos picos R e dos batimentos cardíacos individuais até as classificações do modelo e do exame de 10s final.


