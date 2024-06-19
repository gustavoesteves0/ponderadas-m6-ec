# MNIST
## Rotas

1. Rota `/`

    Método: GET
    Descrição: Envia o arquivo `index.html` localizado na pasta `./templates` para o cliente.
    Retorno: O conteúdo do arquivo `index.html`. Em caso de erro, retorna uma mensagem de erro e o código de status 500.

2. Rota `/train`

    Método: POST
    Descrição: Treina o modelo CNN usando o dataset Fashion MNIST.
        Carrega e normaliza os dados de treinamento e teste.
        Cria o modelo CNN (caso ainda não exista).
        Treina o modelo com os dados de treinamento.
        Salva o modelo treinado em um arquivo chamado `modelo_mnist.h5`.
    Retorno: Um objeto JSON contendo uma mensagem de sucesso e o tempo de treinamento. Em caso de erro, retorna um objeto JSON com a mensagem de erro.

3. Rota `/evaluate`

    Método: GET
    Descrição: Avalia o modelo CNN usando os dados de teste do Fashion MNIST.
        Carrega e normaliza os dados de teste.
        Avalia o modelo carregando os dados de teste.
    Retorno: Um objeto JSON contendo a perda de teste (`test_loss`) e a acurácia de teste (`test_acc`). Em caso de erro, retorna um objeto JSON com a mensagem de erro, indicando que o modelo não foi treinado.

4. Rota `/predict`

    Método: POST
    Descrição: Faz previsões usando o modelo CNN para uma imagem fornecida.
        Recebe uma imagem em formato JSON.
        Normaliza a imagem.
        Faz a predição usando o modelo CNN.
    Retorno: Um objeto JSON contendo a predição completa (`prediction`) e a classe prevista (`predicted_class`). Em caso de erro, retorna um objeto JSON com a mensagem de erro, indicando que o modelo não foi treinado.

## Utilização da solução

ara executar a aplicação, é necessário seguir os seguintes passos:

1. Entrar na raíz do projeto:

```
cd ponderada-MNIST/
```

2. Ligar o venv e instalar os requirements do projeto:

```
source venv/bin/activate
python3 pip install -r requirements.txt
```

3. Executar o código da aplicação:

```
python3 main.py
```
Após a execução desses passos, já é possível acessar a interaface no localhost http://127.0.0.1:5000. 


### Vídeo
O vídeo abaixo serve de demonstração da utilização da solução.

[![V[ideo de demonstração da solução]](https://img.youtube.com/vi/HxsZk-CbJSM/0.jpg)](https://youtu.be/HxsZk-CbJSM)
