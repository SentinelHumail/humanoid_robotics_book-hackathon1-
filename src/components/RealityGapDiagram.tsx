import React from 'react';
import { Target, Cpu } from 'lucide-react';

const RealityGapDiagram = () => {
  return (
    <div style={{
      padding: '2rem',
      background: 'rgba(0,0,0,0.3)',
      borderRadius: '12px',
      border: '1px solid var(--cyber-cyan)',
      margin: '2rem 0',
      display: 'flex',
      flexDirection: 'column',
      alignItems: 'center',
      gap: '2rem'
    }}>
      <div style={{ display: 'flex', justifyContent: 'space-around', width: '100%', alignItems: 'center', flexWrap: 'wrap', gap: '1rem' }}>

        {/* Simulation */}
        <div style={{ textAlign: 'center', flex: '1', minWidth: '200px' }}>
          <div style={{
            width: '80px', height: '80px', margin: '0 auto 1rem',
            background: 'rgba(0, 243, 255, 0.1)', border: '2px solid var(--cyber-cyan)',
            display: 'flex', alignItems: 'center', justifyContent: 'center',
            boxShadow: '0 0 20px rgba(0, 243, 255, 0.4)'
          }}>
            <Cpu size={40} color="var(--cyber-cyan)" />
          </div>
          <h4 style={{ fontFamily: 'Orbitron', color: 'var(--cyber-cyan)' }}>Digital Simulation</h4>
          <p style={{ fontSize: '0.9rem', color: '#aaa' }}>Perfect physics, zero noise, known parameters</p>
        </div>

        {/* The GAP */}
        <div style={{ flex: '0.5', display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
          <div style={{
            height: '2px', width: '100px', background: 'linear-gradient(to right, var(--cyber-cyan), var(--cyber-purple))',
            position: 'relative'
          }}>
            <div style={{
              position: 'absolute', top: '-10px', left: '50%', transform: 'translateX(-50%)',
              color: 'var(--cyber-yellow)', fontFamily: 'Orbitron', fontSize: '0.7rem', fontWeight: 'bold'
            }}>REALITY GAP</div>
          </div>
          <div style={{ fontSize: '2rem', marginTop: '10px' }}>⚡</div>
        </div>

        {/* Reality */}
        <div style={{ textAlign: 'center', flex: '1', minWidth: '200px' }}>
          <div style={{
            width: '80px', height: '80px', margin: '0 auto 1rem',
            background: 'rgba(191, 0, 255, 0.1)', border: '2px solid var(--cyber-purple)',
            display: 'flex', alignItems: 'center', justifyContent: 'center',
            boxShadow: '0 0 20px rgba(191, 0, 255, 0.4)'
          }}>
            <Target size={40} color="var(--cyber-purple)" />
          </div>
          <h4 style={{ fontFamily: 'Orbitron', color: 'var(--cyber-purple)' }}>Physical Reality</h4>
          <p style={{ fontSize: '0.9rem', color: '#aaa' }}>Non-linear friction, sensor noise, dynamic lighting</p>
        </div>
      </div>

      <div style={{
        padding: '1rem', borderTop: '1px dashed rgba(255,255,255,0.1)', width: '100%',
        textAlign: 'center', fontSize: '0.9rem', fontStyle: 'italic', color: 'var(--cyber-yellow)'
      }}>
        "The Simulation is a lie. Reality is the truth."
      </div>
    </div>
  );
};

export default RealityGapDiagram;
