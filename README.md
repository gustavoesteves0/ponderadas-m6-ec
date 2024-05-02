# Atividade Ponderada da Semana 1

## Objetivo
O objetivo da atividade ponderada desenvolvida é a criação de um workspace e um desenho utilizando o turtlesim.  
## Desenho
![image](https://github.com/gustavoesteves0/ponderadas-m6-ec/assets/123904558/7de048cd-74a1-45ac-be2d-49fa9fba3a25)  
O turtlesim vai primeiro desenhar um triângulo, em seguida um círculo e por fim será deletado.  
## Workspace
![image](https://github.com/gustavoesteves0/ponderadas-m6-ec/assets/123904558/61ab0cb4-88b3-4757-a506-71429534687a)  
Acima está a demonstração do workspace, de acordo com os padrões do ROS2. 
## Codigo
![image](https://github.com/gustavoesteves0/ponderadas-m6-ec/assets/123904558/87a64d79-4911-45b4-8fc9-9d67514315b5)
O código acima tem a função de definir as configurações do ambiente e as dependências necessárias para controlar o movimento e as ações da tartaruga virtual no ambiente de simulação.
![image](https://github.com/gustavoesteves0/ponderadas-m6-ec/assets/123904558/129f2cc6-e1d6-4f1f-a0c8-b9f08e1d6311)  
Já o código acima é responsável pela definição da movimentação da tartaruga. No primeiro caso, o movimento é linear, no segundo são definidas as curvas de 60 graus e no último é definida a movimentação em círculo.
![image](https://github.com/gustavoesteves0/ponderadas-m6-ec/assets/123904558/e6b3d984-8b60-4d42-b363-5cafa607f416)  
Já na imagem acima, o código é utilizado para definir as características do traço do desenho, definindo a requisição da cor para o RGB e a grossura do traço, por exemplo.  
![image](https://github.com/gustavoesteves0/ponderadas-m6-ec/assets/123904558/d6b9ba52-c306-41bf-9cfe-d3a95bfd1d72)
Aqui, o código tem a função de definir o formato do desenho, definindo primeiro a criação do triângulo e o aviso de que o triângulo foi feito e, em seguida, a criação do círculo, aviso de criação e delete da tartaruga. Também é importante ressaltar que no começo do código são passadas as informações de cor da caneta, definindo-a como laranja.  
![image](https://github.com/gustavoesteves0/ponderadas-m6-ec/assets/123904558/d981efa5-8e49-461d-a880-cc649f35f7dd)
Aqui é criado o código para excluir a tartaruga.  
![image](https://github.com/gustavoesteves0/ponderadas-m6-ec/assets/123904558/f23e3289-e9e7-44a0-88bc-ede3537ab651)
Por fim, aqui é definida a função main, que é responsável por rodar todo o código.
## Utilizacao da solucao
Após realizar toda a instalação do pacote ROS, o usuário deve realizar o seguinte passo a passo para a utilização do código:
### Primeiro passo:
No primeiro passo, o usuário deve entrar no diretório meu_workspace utilizando o seguinte comando:
```
cd ponderada-1/meu_workspace
```
### Segundo passo:
Em segundo lugar, é necessário fazer a construção do pacote. Para isso, é possível utilizar o comando:
```
colcon build
```
### Terceiro passo:
Após a construção do pacote, é necessário executar o comando:
```
source install/local_setup.bash
```
### Quarto passo:
Deve-se, então, abrir outro terminal e executar o comando para abrir o turtlesim:
```
ros2 run turtlesim turtlesim_node
```
### Quinto passo:
Por fim, para rodar o código, é necessário executar o seguinte comando:
```
ros2 run ponderada_gustavo pond_gu
```
