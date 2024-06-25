class Bluetooth {
  public:
  Bluetooth();
  bool Init();
  char ReadMessage();
  String ReadString();
};