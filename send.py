import socket

def main():
    # 服务器地址和端口
    host = "127.0.0.1"  # 本地主机
    port = 12345        # 与 C++ 服务器端口一致

    # 创建 socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        # 连接到服务器
        client_socket.connect((host, port))
        print("已连接到服务器")

        while True:
            # 从用户获取输入
            user_input = input("> ")
            if user_input.lower() == "exit":
                break

            # 检查输入是否包含 6 个数字
            numbers = user_input.split(",")
            if len(numbers) != 6:
                print("请输入 6 个数字，用逗号分隔！")
                continue

            # 发送数据到服务器
            client_socket.sendall(user_input.encode())

            # 接收服务器返回的数据
            data = client_socket.recv(1024).decode()
            if data:
                print("C++ 程序返回:")
                print(data)

    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 关闭 socket
        client_socket.close()
        print("连接已关闭")

if __name__ == "__main__":
    main()