from flask import Flask, request, jsonify, send_from_directory
import numpy as np
import time
from keras.datasets import fashion_mnist
from keras.utils import to_categorical
from keras.models import Sequential, load_model
from keras.layers import Dense, Conv2D, MaxPool2D, Flatten, Dropout
from keras.optimizers import Adam
import os

app = Flask(__name__)
model = None
training_time = None

def create_model():
    global model
    model = Sequential()
    model.add(Conv2D(filters=32, kernel_size=(3, 3), padding='same', activation='relu', input_shape=(28, 28, 1)))
    model.add(MaxPool2D(pool_size=(2, 2)))
    model.add(Dropout(0.25))
    
    model.add(Conv2D(filters=64, kernel_size=(3, 3), padding='same', activation='relu'))
    model.add(MaxPool2D(pool_size=(2, 2)))
    model.add(Dropout(0.25))
    
    model.add(Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='relu'))
    model.add(MaxPool2D(pool_size=(2, 2)))
    model.add(Dropout(0.25))
    
    model.add(Flatten())
    model.add(Dense(256, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(84, activation='relu'))
    model.add(Dense(10, activation='softmax'))
    
    # Criar um novo otimizador Adam a cada vez
    adam = Adam(learning_rate=0.001)
    model.compile(loss='categorical_crossentropy', metrics=['accuracy'], optimizer=adam)


@app.route('/')
def read_root():
    try:
        return send_from_directory('./templates', 'index.html')
    except Exception as e:
        return str(e), 500

@app.route('/train', methods=['POST'])
def train_model():
    global training_time, model

    # Carregar os dados
    (x_train, y_train), (x_test, y_test) = fashion_mnist.load_data()
    y_train_cat = to_categorical(y_train)
    y_test_cat = to_categorical(y_test)
    x_train_norm = x_train.astype('float32') / 255.0
    x_test_norm = x_test.astype('float32') / 255.0
    x_train_norm = x_train_norm.reshape(len(x_train_norm), 28, 28, 1)
    x_test_norm = x_test_norm.reshape(len(x_test_norm), 28, 28, 1)
    
    if model is None:
        create_model()

    # Treinar o modelo
    try:
        start_time = time.time()
        history = model.fit(x_train_norm, y_train_cat, epochs=20, validation_split=0.2, batch_size=64)
        end_time = time.time()
        training_time = end_time - start_time

        # Salvar o modelo
        model.save('modelo_mnist.h5')

        print(f"Model trained in {training_time} seconds")
        
        return jsonify({'message': 'Model trained', 'training_time': training_time})
    except Exception as e:
        return jsonify({'error': str(e)})

@app.route('/evaluate', methods=['GET'])
def evaluate_model():
    if model is None:
        return jsonify({'error': 'Model not trained'})

    # Carregar os dados de teste
    (_, _), (x_test, y_test) = fashion_mnist.load_data()
    y_test_cat = to_categorical(y_test)
    x_test_norm = x_test.astype('float32') / 255.0
    x_test_norm = x_test_norm.reshape(len(x_test_norm), 28, 28, 1)

    # Avaliar o modelo
    test_loss, test_acc = model.evaluate(x_test_norm, y_test_cat)
    return jsonify({'test_loss': test_loss, 'test_acc': test_acc})

@app.route('/predict', methods=['POST'])
def predict():
    if model is None:
        return jsonify({'error': 'Model not trained'})

    data = request.json
    image = np.array(data['image']).reshape(1, 28, 28, 1)
    image = image.astype('float32') / 255.0  # Normalizar a imagem
    prediction = model.predict(image)
    return jsonify({'prediction': prediction.tolist(), 'predicted_class': int(np.argmax(prediction))})

if __name__ == '__main__':
    if os.path.exists('modelo_mnist.h5'):
        model = load_model('modelo_mnist.h5')
    app.run(host='0.0.0.0', port=5000)
