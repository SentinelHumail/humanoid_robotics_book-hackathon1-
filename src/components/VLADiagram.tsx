import React from 'react';
import { MessageSquare, Camera, ChevronRight, Play } from 'lucide-react';

const VLADiagram = () => {
    return (
        <div style={{
            padding: '2rem',
            background: 'linear-gradient(135deg, rgba(10,10,30,0.9) 0%, rgba(26,10,46,0.9) 100%)',
            borderRadius: '24px',
            border: '1px solid var(--cyber-purple)',
            margin: '2rem 0',
            position: 'relative'
        }}>
            {/* Command Input Overlay */}
            <div style={{
                position: 'absolute', top: '10px', right: '20px', padding: '5px 15px',
                background: 'var(--cyber-yellow)', color: '#000', fontSize: '0.6rem',
                fontFamily: 'Orbitron', fontWeight: 'bold', borderRadius: '4px'
            }}>
                VLA STATUS: ACTIVE
            </div>

            <div style={{ display: 'flex', flexDirection: 'column', gap: '2rem' }}>
                {/* Input Layer */}
                <div style={{ display: 'flex', gap: '1rem', justifyContent: 'center' }}>
                    <div style={{
                        padding: '1rem', background: 'rgba(0, 243, 255, 0.1)', border: '1px solid var(--cyber-cyan)',
                        borderRadius: '12px', display: 'flex', alignItems: 'center', gap: '1rem'
                    }}>
                        <Camera size={20} color="var(--cyber-cyan)" />
                        <span style={{ fontSize: '0.8rem', color: '#fff' }}>Vision (Pix)</span>
                    </div>
                    <div style={{ fontSize: '1.5rem', alignSelf: 'center' }}>+</div>
                    <div style={{
                        padding: '1rem', background: 'rgba(252, 238, 10, 0.1)', border: '1px solid var(--cyber-yellow)',
                        borderRadius: '12px', display: 'flex', alignItems: 'center', gap: '1rem'
                    }}>
                        <MessageSquare size={20} color="var(--cyber-yellow)" />
                        <span style={{ fontSize: '0.8rem', color: '#fff' }}>Language (Text)</span>
                    </div>
                </div>

                {/* The "Brain" Box */}
                <div style={{
                    background: 'rgba(255,255,255,0.02)', border: '1px solid rgba(191, 0, 255, 0.3)',
                    borderRadius: '16px', padding: '1.5rem', textAlign: 'center', position: 'relative'
                }}>
                    <div style={{
                        fontFamily: 'Orbitron', fontSize: '1rem', color: 'var(--cyber-purple)',
                        textShadow: '0 0 10px rgba(191, 0, 255, 0.5)', marginBottom: '1rem'
                    }}>
                        LLM BACKBONE (Action Tokens)
                    </div>
                    <div style={{ display: 'flex', justifyContent: 'center', gap: '0.5rem' }}>
                        {[1, 2, 3, 4, 5, 6].map(i => (
                            <div key={i} style={{
                                width: '30px', height: '10px', background: i < 5 ? 'var(--cyber-purple)' : 'rgba(255,255,255,0.1)',
                                borderRadius: '2px'
                            }} />
                        ))}
                    </div>
                    <div style={{ position: 'absolute', bottom: '-15px', left: '50%', transform: 'translateX(-50%)' }}>
                        <ChevronRight size={30} color="var(--cyber-purple)" />
                    </div>
                </div>

                {/* Final Action Output */}
                <div style={{
                    padding: '1rem', background: 'rgba(5, 5, 10, 0.8)', border: '1px solid #fff',
                    borderRadius: '12px', textAlign: 'center'
                }}>
                    <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', gap: '0.75rem' }}>
                        <Play size={16} color="var(--cyber-cyan)" />
                        <span style={{ fontSize: '0.9rem', color: '#fff', fontFamily: 'Orbitron' }}>
                            MOTOR COMMANDS: [0.12, -0.45, 1.0, 0.0]
                        </span>
                    </div>
                </div>
            </div>
        </div>
    );
};

export default VLADiagram;
