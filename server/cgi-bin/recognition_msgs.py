import pika

data_queue='data_queue'

def send_data_ready(message):
    connection = pika.BlockingConnection(pika.ConnectionParameters(
            host='localhost'))
    channel = connection.channel()
    channel.queue_declare(queue=data_queue, durable=True)
    channel.basic_publish(exchange='',
                          routing_key=data_queue,
                          body=message,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
    print " [x] Sent %r" % (message,)