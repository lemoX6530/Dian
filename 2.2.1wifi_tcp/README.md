### 2.2.1连接网络,与电脑通信

网络密码与SSID须通过idf.py menuconfig进行配置

作为客户端时，板子试图连接menuconfig配置的Ip和端口，在失败十次后则创建服务端

作为服务端时，同一局域网下的设备可以连接LOG中给出的ip和menuconfig配置的服务端端口
      
向esp板子发送字符串，则esp回复以同样内容
