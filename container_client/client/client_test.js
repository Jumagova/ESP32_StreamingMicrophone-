const net = require('net');
const fs = require('fs');

const socket = new net.Socket();

socket.connect(8080, '192.168.1.30', () => {
    console.log('Connected to the server');

    const readStream = fs.createReadStream('test_sounds/piano2.wav');
    readStream.pipe(socket);

    readStream.on('end', () => {
        console.log('Finished sending data');
        socket.end();
    });
});

socket.on('close', () => {
    console.log('Connection closed');
});