const http = require("http");
const socketIO = require("socket.io");
const ss = require('socket.io-stream')
const fs = require("fs");

const httpServer = http.createServer();
const io = new socketIO.Server(httpServer, {
  cors: {
    origin: "http://localhost:3000",
    methods: ["GET", "POST"],
    allowedHeaders: ["my-custom-header"],
    credentials: true
  }
});

io.on("connection", (socket) => {
  // ...
  console.log('Client connected');

  socket.on('client-stream-request', (data) =>{
    let stream = ss.createStream();
    let filename = __dirname + './test_sounds/organfinale.mp3'
    ss(socket).emit('audio-stream' , stream, {name:filename});
    fs.createReadStream(filename).pipe(stream)
  });
  // setInterval(()=> {
  //   socket.emit('datos', 'dato prueba')
  // }, 3000);
  

  // const audioStream = fs.createReadStream('./test_sounds/organfinale.mp3');

  // audioStream.on('error', (err) => {
  //   console.error(`Error reading audio file: ${err}`);
  //   socket.disconnect();
  // });

  // audioStream.on('data', (chunk) => {
  //   console.log(chunk); // Add this line to log the chunk data
  //   socket.emit('audio_data', chunk);
  // });

  // audioStream.on('end', () => {
  //   console.log('Audio file sent to client');
  //   // socket.disconnect();
  // });

  socket.on('disconnect', () => {
    console.log('Client disconnected');
  });
});

console.log('Server Listening on port 3002');
httpServer.listen(3002);

