from tensorflow.keras.models import load_model
from PIL import Image
import numpy as np
modelo = load_model('modelo_mnist.h5')
imagem = Image.open('./content/imagem01.jpg')
imagem = imagem.resize((28, 28))
imagem = imagem.convert('L')
imagem = np.array(imagem) / 255.0
imagem = imagem.reshape(1, 28, 28, 1)
predict = modelo.predict(imagem)
predict = np.argmax(predict)
print(predict)