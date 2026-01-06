import React from 'react';
import { Eye, Brain, Cpu, Layers } from 'lucide-react';

const PerceptionPipelineDiagram = () => {
    return (
        <div style={{
            padding: '2rem',
            background: 'rgba(10, 10, 30, 0.8)',
            borderRadius: '20px',
            border: '2px solid var(--cyber-cyan)',
            margin: '2rem 0',
            boxShadow: '0 0 30px rgba(0, 243, 255, 0.2)'
        }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: '1rem', flexWrap: 'wrap' }}>
                {/* Step 1: Input */}
                <div style={{ flex: 1, minWidth: '150px', display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
                    <div style={{
                        width: '60px', height: '60px', borderRadius: '50%', background: 'rgba(0, 243, 255, 0.1)',
                        border: '1px solid var(--cyber-cyan)', display: 'flex', alignItems: 'center', justifyContent: 'center',
                        marginBottom: '1rem'
                    }}>
                        <Eye color="var(--cyber-cyan)" size={30} />
                    </div>
                    <span style={{ fontSize: '0.8rem', fontFamily: 'Orbitron', color: 'var(--cyber-cyan)' }}>RAW INPUT</span>
                    <span style={{ fontSize: '0.7rem', color: '#666' }}>RGB-D / Stereo</span>
                </div>

                <div style={{ color: 'var(--cyber-cyan)', opacity: 0.5 }}>➜</div>

                {/* Step 2: Accelerate */}
                <div style={{ flex: 1, minWidth: '150px', display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
                    <div style={{
                        width: '60px', height: '60px', borderRadius: '10px', background: 'rgba(191, 0, 255, 0.1)',
                        border: '1px solid var(--cyber-purple)', display: 'flex', alignItems: 'center', justifyContent: 'center',
                        marginBottom: '1rem'
                    }}>
                        <Cpu color="var(--cyber-purple)" size={30} />
                    </div>
                    <span style={{ fontSize: '0.8rem', fontFamily: 'Orbitron', color: 'var(--cyber-purple)' }}>NITROS</span>
                    <span style={{ fontSize: '0.7rem', color: '#666' }}>Zero-Copy GPU</span>
                </div>

                <div style={{ color: 'var(--cyber-purple)', opacity: 0.5 }}>➜</div>

                {/* Step 3: Neural Model */}
                <div style={{ flex: 1, minWidth: '150px', display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
                    <div style={{
                        width: '60px', height: '60px', background: 'rgba(252, 238, 10, 0.1)',
                        border: '1px solid var(--cyber-yellow)', display: 'flex', alignItems: 'center', justifyContent: 'center',
                        marginBottom: '1rem', transform: 'rotate(45deg)'
                    }}>
                        <div style={{ transform: 'rotate(-45deg)' }}>
                            <Brain color="var(--cyber-yellow)" size={30} />
                        </div>
                    </div>
                    <span style={{ fontSize: '0.8rem', fontFamily: 'Orbitron', color: 'var(--cyber-yellow)' }}>TENSORRT</span>
                    <span style={{ fontSize: '0.7rem', color: '#666' }}>Inference</span>
                </div>

                <div style={{ color: 'var(--cyber-yellow)', opacity: 0.5 }}>➜</div>

                {/* Step 4: Semantic Output */}
                <div style={{ flex: 1, minWidth: '150px', display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
                    <div style={{
                        width: '60px', height: '60px', borderRadius: '4px', background: 'rgba(255, 255, 255, 0.05)',
                        border: '1px solid #fff', display: 'flex', alignItems: 'center', justifyContent: 'center',
                        marginBottom: '1rem'
                    }}>
                        <Layers color="#fff" size={30} />
                    </div>
                    <span style={{ fontSize: '0.8rem', fontFamily: 'Orbitron', color: '#fff' }}>POSE / MAP</span>
                    <span style={{ fontSize: '0.7rem', color: '#666' }}>Semantic Logic</span>
                </div>
            </div>

            <div style={{
                marginTop: '2rem', padding: '0.75rem', background: 'rgba(0,0,0,0.4)', borderRadius: '8px',
                borderLeft: '4px solid var(--cyber-yellow)', fontSize: '0.85rem', color: '#ddd'
            }}>
                <span style={{ color: 'var(--cyber-yellow)', fontWeight: 'bold' }}>SYSTEM LOG:</span> Pipeline latency optimized to 3.4ms via NITROS hardware acceleration.
            </div>
        </div>
    );
};

export default PerceptionPipelineDiagram;
