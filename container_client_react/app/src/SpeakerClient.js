

// // import React, { useEffect, useRef, useState } from 'react';
// // import {Container, Row, Col } from 'react-bootstrap';
// // import io from 'socket.io-client';
// // import wavDecoder from 'wav-decoder';

// // const Client = () => {
// //   const [audio, setAudio] = useState(null);
// //   const [Connected, setConnected] = useState(false);
// //   const socketRef = useRef(null);

// //   useEffect(() => {
// //     const socket = io('http://localhost:3002');

// //     socket.addEventListener('open', () => {
// //       console.log('Connected to server!');
// //       setConnected(true);
// //     });

// //     socket.on('datos', (data) =>{
// //       console.log(data);
// //     })

// //     socket.on('audio_data', (event) => {
// //       //Parse Wav data
// //       console.log(event);
// //       // const view = new Uint8Array (event);
// //       // console.log(view);
// //       const audioData = new Uint8Array(event);
// //       // console.log(audioData);
// //       // const wavData = wavDecoder.decode(event);
// //       // // Convert the WAV data to a blob and set it as the audio state
// //       const audioBlob = new Blob([new Float32Array(audioData)], { type: 'audio/wav' });
// //       setAudio(audioBlob);
// //     });

// //     // socket.addEventListener('close', () => {
// //     //   console.log('Disconnected from server!');
// //     //   setConnected(false);
// //     //   setAudio(null);
// //     // });

// //     socketRef.current = socket;

// //     return () => {
// //       // if (socketRef.current) {
// //       //   socketRef.current.close();
// //       // }
// //     };
// //   }, []);

// //   // Use the hook and render the audio element
// //   function AudioPlayer({ audio }) {
// //     const [url, setUrl] = useState(null);

// //     useEffect(() => {
// //       if (audio) {
// //         setUrl(URL.createObjectURL(audio));
// //       }
// //       return () => {
// //         if (url) {
// //           URL.revokeObjectURL(url);
// //         }
// //       };
// //     }, [audio]);

// //     return url ? <audio controls src={url} /> : null;
// //   }

// //   // function useObjectUrl(audio) {
// //   //   const url = useMemo(() => URL.createObjectURL(audio), [audio]);
// //   //   useEffect(() => () => URL.revokeObjectURL(url), [audio]);
// //   //   return url;
// //   // }

// //   return (
// //     <Container>
// //       <Row>
// //         <Col>
// //           <h2>Audio player using binary data</h2>
// //           {audio ? (
// //             <AudioPlayer {...{ audio }} />
// //           ) : (
// //             <div>There was an error fetching the audio file</div>
// //           )}
// //         </Col>
// //       </Row>
// //     </Container>
// //   );
// // };

// // export default Client;

// // import React, { useEffect, useRef, useState } from 'react';
// // import { Container, Row, Col } from 'react-bootstrap';
// // import io from 'socket.io-client';
// // import ss from 'socket.io-stream';
// // import wavDecoder from 'wav-decoder';

// // const Client = () => {
// //   // const [audio, setAudio] = useState(null);
// //   const [connected, setConnected] = useState(false);
// //   const socketRef = useRef(null);
// //   let audio = document.getElementById('player')
  
// //   useEffect(() => {
// //     const socket = io.connect('http://localhost:3002');

// //     socket.on('connect', () => {
// //       new Notification("Connected to server!");
// //       setConnected(true);
// //     });

// //     // socket.on('audio_data', (event) => {
// //     //   const view = new Uint8Array(event);
// //     //   console.log(view);
// //     //   wavDecoder.decode(event).then((audioData) => {
// //     //     const audioBlob = new Blob([audioData.channelData], { type: 'audio/wav' });
// //     //     setAudio(audioBlob);
// //     //   }).catch((err) => {
// //     //     console.error(`Error decoding audio data: ${err}`);
// //     //   });
// //     // });


// //     // socketRef.current = socket;

// //     ss(socket).on('audio-stream', (stream, data) => {
// //       let parts = [];
// //       stream.on('data', function(chunk){
// //           parts.push(chunk);
// //       });
// //       stream.on('end',  () => {
// //           audio.src = (window.URL || window.webkitURL).createObjectURL(new Blob(parts));
// //           audio.play();
// //       });
// //   });

// //     return () => {
// //       if (socketRef.current) {
// //         socketRef.current.close();
// //       }
// //     };
// //   }, []);

// //   // const AudioPlayer = ({ audio }) => {
// //   //   const [url, setUrl] = useState(null);

// //   //   useEffect(() => {
// //   //     if (audio) {
// //   //       setUrl(URL.createObjectURL(audio));
// //   //     }
// //   //     return () => {
// //   //       if (url) {
// //   //         URL.revokeObjectURL(url);
// //   //       }
// //   //     };
// //   //   }, [audio]);

// //   //   return url ? <audio controls src={url} /> : null;
// //   // };

// //   return (
// //     <Container>
// //       <Row>
// //         <Col>
// //           <h2>Audio player using binary data</h2>
// //           {audio ? (
// //             <AudioPlayer audio={audio} />
// //           ) : (
// //             <div>There was an error fetching the audio file</div>
// //           )}
// //         </Col>
// //       </Row>
// //     </Container>
// //   );
// // };

// // export default Client;

import React, { useEffect, useRef, useState } from 'react';
import { Container, Row, Col } from 'react-bootstrap';
import io from 'socket.io-client';
import ss from 'socket.io-stream';
import wavDecoder from 'wav-decoder';


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

    ss(socket).on('audio-stream', (stream, data) => {
      const parts = [];
      stream.on('data', function(chunk){
          parts.push(chunk);
      });
      stream.on('end',  () => {
          const blob = new Blob(parts, { type: 'audio/mp3' });
          setAudio(blob);
      });
    });

    socketRef.current = socket;

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

