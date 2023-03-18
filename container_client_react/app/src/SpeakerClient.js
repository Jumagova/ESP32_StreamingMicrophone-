

import React, { useEffect, useRef, useState } from 'react';
import { Container, Row, Col } from 'react-bootstrap';
import io from 'socket.io-client';



const Client = () => {
  const [connected, setConnected] = useState(false);
  const [audio, setAudio] = useState(null);
  const socketRef = useRef(null);

  useEffect(() => {
    const socket = io.connect('http://localhost:3002');

    socket.on('connect', () => {
      new Notification('Connected to server!');
      setConnected(true);
    });

    socket.addEventListener('open', () => {
      console.log('Connected to server!');
      setConnected(true);
    });

    const parts = [];

    socket.on('audio_data', (chunk) => {
      parts.push(chunk);
    });

    socket.on('audio_end', () => {
      const blob = new Blob(parts, { type: 'audio/mp3' });
      setAudio(blob);
    });

    socketRef.current = socket;

    socket.addEventListener('close', () => {
      console.log('Disconnected from server!');
      setConnected(false);
    });

    return () => {
      if (socketRef.current) {
        socketRef.current.close();
      }
    };
  }, []);

  return (
    <Container>
      <Row>
        <Col>
          <h2>Audio player using binary data</h2>
          {audio ? (
            <audio controls src={URL.createObjectURL(audio)} />
          ) : (
            <div>There was an error fetching the audio file</div>
          )}
        </Col>
      </Row>
    </Container>
  );
};

export default Client;

