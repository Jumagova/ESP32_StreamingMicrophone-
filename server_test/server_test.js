const net = require('net');
const os = require('os');
const Speaker = require('speaker');

const server = net.createServer(socket => {
    console.log('Client connected');

    let speaker;

    socket.on('error', error => {
        console.error('Socket error: ', error);
        speaker.end();
    });

    socket.on('data', data => {
        //console.log('Received data: ', data);

        // Play the audio data using the speaker library 
        // For Esp32
        if (!speaker) {
            speaker = new Speaker({
                channels: 2,
                bitDepth: 16,
                sampleRate: 2250
            });
        }
        // For client_test
        // if (!speaker) {
        //     speaker = new Speaker({
        //         channels: 2,
        //         bitDepth: 16,
        //         sampleRate: 44100
        //     });
        // }

        speaker.write(data);
    });

    socket.on('end', () => {
        console.log('Client disconnected');
        if (speaker) {
            speaker.end();
        }
    });
});

server.listen(3030, () => {
    console.log('Server is listening on port 3030');

    // Get the IP address of the server
    const interfaces = os.networkInterfaces();
    const addresses = [];
    for (const k in interfaces) {
        for (const k2 in interfaces[k]) {
            const address = interfaces[k][k2];
            if (address.family === 'IPv4' && !address.internal) {
                addresses.push(address.address);
            }
        }
    }

    // Log the IP addresses of the server
    console.log('Server IP addresses: ', addresses);
});