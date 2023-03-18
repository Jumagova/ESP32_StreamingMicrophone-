const http = require("http");
const socketIO = require("socket.io");
const ss = require('socket.io-stream')
const fs = require("fs");

const httpServer = http.createServer();
const io = new socketIO.Server(httpServer, {
  cors: {
    origin: "*",
  }
});

io.on("connection", (socket) => {
  console.log('Client connected');
  const audioStream = fs.createReadStream('./test_sounds/CantinaBand60.wav');

  audioStream.on('error', (err) => {
    console.error(`Error reading audio file: ${err}`);
    socket.disconnect();
  });

  audioStream.on('data', (chunk) => {
    console.log(chunk); // Add this line to log the chunk data
    socket.emit('audio_data', chunk);
  });

  audioStream.on('end', () => {
    console.log('Audio file sent to client');
    // socket.disconnect();
    socket.emit('audio_end')
  });

  socket.on('disconnect', () => {
    console.log('Client disconnected');
  });
});

console.log('Server Listening on port 3002');
httpServer.listen(3002);

