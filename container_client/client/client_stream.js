const net = require('net');
const Speaker = require('speaker');

const Client = () => {
const [message, setMessage] = useState('');
const [connected, setConnected] = useState(false);
const socketRef = useRef(null);

useEffect(() => {
    socketRef.current = net.connect({ host: 'your-server-hostname', port: 8080 }, () => {
    console.log('Connected to server!');
    setConnected(true);
    });

    socketRef.current.on('data', (data) => {
    const message = data.toString();
    setMessage(message);
    });

    socketRef.current.on('end', () => {
    console.log('Disconnected from server!');
    setConnected(false);
    });

    return () => {
    if (socketRef.current) {
        socketRef.current.end();
    }
    };
}, []);

const handleSend = () => {
    if (socketRef.current && connected) {
    socketRef.current.write('Hello, server!');
    }
};

return (
    <Container>
    <Row>
        <Col>
        <p>Message from server: {message}</p>
        </Col>
    </Row>
    <Row>
        <Col>
        <Button onClick={handleSend} disabled={!connected}>
            Send message
        </Button>
        </Col>
    </Row>
    </Container>
);
};

export default Client;
