import 'dart:async';
import 'dart:convert';
import 'package:mqtt_client/mqtt_client.dart' as mqtt;
import 'package:flutter/material.dart';

void main() => runApp(MyHomePage());

class MyHomePage extends StatefulWidget {
  @override
  _MyHomePageState createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  PageController _pageController;

  final roomController = TextEditingController();
  final xValueController = TextEditingController();
  final yValueController = TextEditingController();

  @override
  void dispose() {
    // Clean up the controller when the Widget is disposed
    roomController.dispose();
    xValueController.dispose();
    yValueController.dispose();
    super.dispose();
  }

  String broker = 'eu.thethings.network';
  //String broker = 'labict.be';

  mqtt.MqttClient client;
  mqtt.MqttConnectionState connectionState;
  Set<String> topics = Set<String>();
  StreamSubscription subscription;

  String messageFromMqtt =
      '{"app_id":"locationtracking","dev_id":"locationdevice1","hardware_serial":"00CE3CFCB095D853","port":1,"counter":5,"payload_raw":"AQ==",'
      '"metadata":{"time":"2019-01-28T16:33:13.522531229Z","frequency":867.1,"modulation":"LORA","data_rate":"SF7BW125","airtime":46336000,"coding_rate":"4/5",'
      '"gateways":['
      '{"gtw_id":"eui-1dee09c05d572b28","timestamp":1179517907,"time":"","channel":3,"rssi":-107,"snr":5.8,"rf_chain":0,"latitude":51.193756,"longitude":3.2183638,"altitude":10,"location_source":"registry"},'
      '{"gtw_id":"vives-ttn-03","gtw_trusted":true,"timestamp":3587962947,"time":"2019-01-28T16:33:17Z","channel":3,"rssi":-84,"snr":8.75,"rf_chain":0},'
      '{"gtw_id":"vives-ttn-01","gtw_trusted":true,"timestamp":1142609643,"time":"2019-01-28T16:33:13Z","channel":3,"rssi":-69,"snr":9,"rf_chain":0}]}}';
  //Map jsonMap = JSON.decode(messageFromMqtt);
  //Map<String, dynamic> user = jsonDecode(messageFromMqtt);
  //dynamic convert(String input) => _parseJson(input, _reviver);

  Map<String, dynamic> jsonMQTT;

  int _page = 0;
  int gateway1 = 0;
  int gateway2 = 0;
  int gateway3 = 0;

  var x;
  var y;

  String roomMeter = "";
  int xValueMeter = 0;
  int yValueMeter = 0;

  void addValuesToArray() {}

  void locationAlgorithm() {}

  void _connect() async {
    /// First create a client, the client is constructed with a broker name, client identifier
    /// and port if needed. The client identifier (short ClientId) is an identifier of each MQTT
    /// client connecting to a MQTT broker. As the word identifier already suggests, it should be unique per broker.
    /// The broker uses it for identifying the client and the current state of the client. If you don’t need a state
    /// to be hold by the broker, in MQTT 3.1.1 you can set an empty ClientId, which results in a connection without any state.
    /// A condition is that clean session connect flag is true, otherwise the connection will be rejected.
    /// The client identifier can be a maximum length of 23 characters. If a port is not specified the standard port
    /// of 1883 is used.
    /// If you want to use websockets rather than TCP see below.

    jsonMQTT = jsonDecode(messageFromMqtt);

    client = mqtt.MqttClient(broker, '');

    //Try for password but not found it but in mqtt_client.dart can you work a round a give there your pass and username.
    //mqtt.connectionMessage.authenticateAs(username, password);
    //mqtt.connect([String username, String password]);

    /// A websocket URL must start with ws:// or wss:// or Dart will throw an exception, consult your websocket MQTT broker
    /// for details.
    /// To use websockets add the following lines -:
    // client.useWebSocket = true;

    /// This flag causes the mqtt client to use an alternate method to perform the WebSocket handshake. This is needed for certain
    /// matt clients (Particularly Amazon Web Services IOT) that will not tolerate additional message headers in their get request
    // client.useAlternateWebSocketImplementation = true;
    // client.port = 443; // ( or whatever your WS port is)
    /// Note do not set the secure flag if you are using wss, the secure flags is for TCP sockets only.

    /// Set logging on if needed, defaults to off
    client.logging(on: true);

    /// If you intend to use a keep alive value in your connect message that is not the default(60s)
    /// you must set it here
    client.keepAlivePeriod = 30;

    /// Add the unsolicited disconnection callback
    client.onDisconnected = _onDisconnected;

    /// Create a connection message to use or use the default one. The default one sets the
    /// client identifier, any supplied username/password, the default keepalive interval(60s)
    /// and clean session, an example of a specific one below.
    final mqtt.MqttConnectMessage connMess = mqtt.MqttConnectMessage()
        .withClientIdentifier('Mqtt_MyClientUniqueId2')
        // Must agree with the keep alive set above or not set
        .startClean() // Non persistent session for testing
        .keepAliveFor(30)
        // If you set this you must set a will message
        .withWillTopic('willtopic')
        .withWillMessage('My Will message')
        .withWillQos(mqtt.MqttQos.atLeastOnce);
    print('MQTT client connecting....');
    client.connectionMessage = connMess;

    /// Connect the client, any errors here are communicated by raising of the appropriate exception. Note
    /// in some circumstances the broker will just disconnect us, see the spec about this, we however will
    /// never send malformed messages.
    try {
      await client.connect();
    } catch (e) {
      print(e);
      _disconnect();
    }

    /// Check if we are connected
    if (client.connectionState == mqtt.MqttConnectionState.connected) {
      print('MQTT client connected');
      setState(() {
        connectionState = client.connectionState;
      });
    } else {
      print('ERROR: MQTT client connection failed - '
          'disconnecting, state is ${client.connectionState}');
      _disconnect();
    }

    /// The client has a change notifier object(see the Observable class) which we then listen to to get
    /// notifications of published updates to each subscribed topic.
    subscription = client.updates.listen(_onMessage);

    _subscribeToTopic("locationtracking/devices/locationdevice1/up");
  }

  void _disconnect() {
    client.disconnect();
    _onDisconnected();
  }

  void _onDisconnected() {
    setState(() {
      topics.clear();
      connectionState = client.connectionState;
      client = null;
      subscription.cancel();
      subscription = null;
    });
    print('MQTT client disconnected');
  }

  void _onMessage(List<mqtt.MqttReceivedMessage> event) {
    print(event.length);
    final mqtt.MqttPublishMessage recMess =
        event[0].payload as mqtt.MqttPublishMessage;
    final String message =
        mqtt.MqttPublishPayload.bytesToStringAsString(recMess.payload.message);

    setState(() {
      this.messageFromMqtt = message;
    });

    /// The above may seem a little convoluted for users only interested in the
    /// payload, some users however may be interested in the received publish message,
    /// lets not constrain ourselves yet until the package has been in the wild
    /// for a while.
    /// The payload is a byte buffer, this will be specific to the topic
    print('MQTT message: topic is <${event[0].topic}>, '
        'payload is <-- ${message} -->');
    setState(() {});
  }

  void _subscribeToTopic(String topic) {
    if (connectionState == mqtt.MqttConnectionState.connected) {
      client.subscribe(topic, mqtt.MqttQos.exactlyOnce);
      setState(() {});
    }
  }

  void _unsubscribeFromTopic(String topic) {
    if (connectionState == mqtt.MqttConnectionState.connected) {
      setState(() {
        client.unsubscribe(topic);
      });
    }
  }

  _onTapDown(TapDownDetails details) {
    x = details.globalPosition.dx;
    y = details.globalPosition.dy;
    print("tap down " + x.toString() + ", " + y.toString());

  }

  _onTapUp(TapUpDetails details) {
    x = details.globalPosition.dx;
    y = details.globalPosition.dy;
    print("tap up " + x.toString() + ", " + y.toString());
  }

  @override
  Widget build(BuildContext context) {


    IconData connectionStateIcon;
    switch (client?.connectionState) {
      case mqtt.MqttConnectionState.connected:
        connectionStateIcon = Icons.cloud_done;
        break;
      case mqtt.MqttConnectionState.disconnected:
        connectionStateIcon = Icons.cloud_off;
        break;
      case mqtt.MqttConnectionState.connecting:
        connectionStateIcon = Icons.cloud_upload;
        break;
      case mqtt.MqttConnectionState.disconnecting:
        connectionStateIcon = Icons.cloud_download;
        break;
      case mqtt.MqttConnectionState.faulted:
        connectionStateIcon = Icons.error;
        break;
      default:
        connectionStateIcon = Icons.cloud_off;
    }



    return MaterialApp(
      debugShowCheckedModeBanner: false,
      home: Scaffold(
        appBar: AppBar(
          title: Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              Text("Indoor location"),
              SizedBox(
                width: 8.0,
              ),
              Icon(connectionStateIcon),
              SizedBox(height: 8.0),
              RaisedButton(
                child: Text(client?.connectionState ==
                        mqtt.MqttConnectionState.connected
                    ? 'Disconnect'
                    : 'Connect'),
                onPressed: () {
                  if (client?.connectionState ==
                      mqtt.MqttConnectionState.connected) {
                    _disconnect();
                  } else {
                    _connect();
                  }
                },
              ),
            ],
          ),
        ),
        bottomNavigationBar: BottomNavigationBar(
          items: <BottomNavigationBarItem>[
            BottomNavigationBarItem(
                icon: Icon(Icons.add_location), title: Text('New location')),
            BottomNavigationBarItem(
                icon: Icon(Icons.location_searching), title: Text('Locate')),
            BottomNavigationBarItem(
                icon: Icon(Icons.map), title: Text('Map')),
          ],
          currentIndex: _page,
          fixedColor: Colors.blue,
          onTap: navigationTapped,
        ),
        body: PageView(
          controller: _pageController,
          onPageChanged: onPageChanged,
          children: <Widget>[
            _buildNewLocationPage(),
            _buildLocatePage(),
            _buildNewMapPage(),
          ],
        ),
      ),
    );
  }

  Column _buildNewLocationPage() {
    final _formKey = GlobalKey<FormState>();
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: <Widget>[
        new ListTile(
          leading: const Icon(Icons.location_city),
          title: new TextField(
            controller: roomController,
            decoration: new InputDecoration(
              hintText: "Classroom",
            ),
          ),
        ),
        new ListTile(
          leading: const Icon(Icons.more_horiz),
          title: new TextField(
            controller: xValueController,
            decoration: new InputDecoration(
              hintText: "X-waarde",
            ),
            keyboardType: TextInputType.number,
          ),
        ),
        new ListTile(
          leading: const Icon(Icons.more_vert),
          title: new TextField(
            controller: yValueController,
            decoration: new InputDecoration(
              hintText: "Y-waarde",
            ),
            keyboardType: TextInputType.number,
          ),
        ),
        RaisedButton(
          onPressed: () {
            addValuesToArray();
          },
          child: Text("Add location"),textColor: Colors.white,
          color: Colors.cyanAccent,
        ),
        gatewayRSSI(),
      ],
    );
  }

  Column _buildLocatePage() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: <Widget>[
        new ListTile(
          leading: const Icon(Icons.location_city),
          title: new Text('Classroom: ${roomMeter}'),
        ),
        new ListTile(
          leading: const Icon(Icons.more_horiz),
          title: new Text('X-waarde: ${xValueMeter} m'),
        ),
        new ListTile(
          leading: const Icon(Icons.more_vert),
          title: new Text('Y-waarde: ${yValueMeter} m'),
        ),
        RaisedButton(
          onPressed: () {
            locationAlgorithm();
          },
          child: Text("Get location"),textColor: Colors.white,
          color: Colors.greenAccent,
        ),
        gatewayRSSI(),
      ],
    );
  }
  Column _buildNewMapPage() {

    return Column (
          children: <Widget>[

            new ListTile(
              title: new Text('x-waarde: ${x.toString()}'),
            ),
            new ListTile(
              title: new Text('y-waarde: ${y.toString()}'),
            ),
            RaisedButton(
              onPressed: () {
                locationAlgorithm();
              },
              child: Text("Get location"),textColor: Colors.white,
              color: Colors.red,
            ),

            RaisedButton(
              onPressed: () {
                addValuesToArray();
              },
              child: Text("Add location"),textColor: Colors.white,
              color: Colors.red,
            ),
            Image.asset(
              'lib/assets/plattegrond.jpg',
            ),

            GestureDetector(
              onTap: () => print('tapped!'),
              onTapDown: (TapDownDetails details) => _onTapDown(details),
              onTapUp: (TapUpDetails details) => _onTapUp(details),

            ),


          ],

    );

  }



  Column gatewayRSSI() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: <Widget>[
        RaisedButton(
          onPressed: () {
            jsonMQTT = jsonDecode(messageFromMqtt);
            setState(() {
              gateway1 = jsonMQTT["metadata"]['gateways'][0]["rssi"];
              gateway2 = jsonMQTT["metadata"]['gateways'][1]["rssi"];
              gateway3 = jsonMQTT["metadata"]['gateways'][2]["rssi"];
            });
          },

            //width: double.infinity,
            child: Text('Get RSSI'),textColor: Colors.white,
          color: Colors.redAccent,

        ),
        new ListTile(
          leading: const Icon(Icons.signal_cellular_4_bar),
          title: new Text('Gateway 1: $gateway1 dBm'),
        ),
        new ListTile(
          leading: const Icon(Icons.signal_cellular_4_bar),
          title: new Text('Gateway 2: $gateway2 dBm'),
        ),
        new ListTile(
          leading: const Icon(Icons.signal_cellular_4_bar),
          title: new Text('Gateway 3: $gateway3 dBm'),
        ),
      ],
    );
  }



  void navigationTapped(int page) {
    _pageController.animateToPage(page,
        duration: const Duration(milliseconds: 400), curve: Curves.ease);
  }

  void onPageChanged(int page) {
    setState(() {
      this._page = page;
    });
  }

  @override
  void initState() {
    _pageController = PageController();
    super.initState();
  }
}
