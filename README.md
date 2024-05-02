# Atividade Ponderada da Semana 1

## Objetivo
O objetivo da atividade ponderada desenvolvida eh a criacao de um workspace e um desenho utilizando o turtlesim. 
## Desenho
![image](https://github.com/gustavoesteves0/ponderadas-m6-ec/assets/123904558/7de048cd-74a1-45ac-be2d-49fa9fba3a25)  
O turtlesim vai primeiro desenhar um triangulo, em seguida um circulo e por fim sera deletado.
## Workspace
![image](https://github.com/gustavoesteves0/ponderadas-m6-ec/assets/123904558/61ab0cb4-88b3-4757-a506-71429534687a)  
Acima esta a demonstracao do workspace, de acordo com os padroes do ROS2.
## Codigo
![image](https://github.com/gustavoesteves0/ponderadas-m6-ec/assets/123904558/87a64d79-4911-45b4-8fc9-9d67514315b5)
O codigo acima tem a funcao de definir as configuracoes do ambiente e as dependencias necessarias para controlar o movimento e as ações da tartaruga virtual no ambiente de simulação.  
![image](https://github.com/gustavoesteves0/ponderadas-m6-ec/assets/123904558/129f2cc6-e1d6-4f1f-a0c8-b9f08e1d6311)  
Ja o codigo acima eh responsavel pela definicao da movimentacao da tartaruga. No primeiro caso, o movimento eh linear, no segundo sao definidas as curvas de 60 graus e no ultimo eh definida a movimentacao em circulo.  
![image](https://github.com/gustavoesteves0/ponderadas-m6-ec/assets/123904558/e6b3d984-8b60-4d42-b363-5cafa607f416)  
Ja na imagem acima, o codigo eh utilizado para definir as caracteristicas do traco do desenho, definindo a requisicao da cor para o RGB e a grossura do traco, por exemplo.  
![image](https://github.com/gustavoesteves0/ponderadas-m6-ec/assets/123904558/d6b9ba52-c306-41bf-9cfe-d3a95bfd1d72)
Aqui, o codigo tem a funcao de definir o formato do desenho, definindo primeiro a criacao do triangulo e o aviso de que o triangulo foi feito e, em seguida, a criacao do circulo, aviso de criacao e delete da tartaruga. Tambem eh importante ressaltar que no comeco do codigo sao passadas as informacoes de cor da caneta, definindo-a como laranja.
![image](https://github.com/gustavoesteves0/ponderadas-m6-ec/assets/123904558/d981efa5-8e49-461d-a880-cc649f35f7dd)
Aqui eh criado o codigo para excluir a tartaruga.
![image](https://github.com/gustavoesteves0/ponderadas-m6-ec/assets/123904558/f23e3289-e9e7-44a0-88bc-ede3537ab651)
Por fim, aqui eh definida a funcao main, que eh responsavel por rodar todo o codigo.
## Utilizacao da solucao
Apos realizar toda a instalcao do pacote ROS, o usuario deve realizar o seguinte passo a passo para a utilizacao do codigo:
### Primeiro passo:
No primeiro passo, o usuario deve entrar no diretorio meu_workspace utilizando o seguinte comando:
```
cd ponderada-1/meu_workspace
```
### Segundo passo:
Em segundo lugar, eh necessario fazer a construcao do pacote. Para isso, eh possivel utilizar o comando:
```
colcon build
```
### Terceiro passo:
Apos a construcao do pacote, eh necessario executar o comando:
```
source install/local_setup.bash
```
### Quarto passo:
Deve-se, entao, abrir outro terminal e executar o comando para abrir o turtlesim:
```
ros2 run turtlesim turtlesim_node
```
### Quinto passo:
Por fim, para rodar o codigo, eh necessario executar o seguinte comando:
```
ros2 run ponderada_gustavo pond_gu
```
