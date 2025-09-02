import websocket
import threading
import time

def on_message(ws, message):
    print("Mensagem recebida:", message)

def on_error(ws, error):
    print("Erro:", error)

def on_close(ws, close_status_code, close_msg):
    print("Conexão fechada.")

def on_open(ws):
    print("Conexão estabelecida!")
    # Envie uma mensagem para teste (o servidor deverá ecoar ou processar conforme sua implementação)
    ws.send("DEPLOY=true")

if __name__ == "__main__":
    # Se desejar ver detalhes de debug
    websocket.enableTrace(True)

    # Atualize a URL para apontar para o endereço e a porta onde o seu MOOS (servidor) estará escutando.
    ws_url = "ws://localhost:8000"  # Exemplo: servidor escutando na porta 8000

    # Cria o objeto WebSocketApp com os callbacks configurados
    ws = websocket.WebSocketApp(
        ws_url,
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close
    )

    # Executa o WebSocket em uma thread separada para evitar bloquear o fluxo principal
    ws_thread = threading.Thread(target=ws.run_forever)
    ws_thread.start()

    # Mantenha o cliente ativo por um tempo para trocar mensagens (por exemplo, 20 segundos)
    time.sleep(20)

    # Fecha a conexão e aguarda a finalização da thread
    ws.close()
    ws_thread.join()
    print("Cliente finalizado.")
