import React from 'react';
import { Share2, Zap, MessageSquare, Activity } from 'lucide-react';

const ROS2NervousSystemDiagram = () => {
    return (
        <div style={{
            padding: '2rem',
            background: 'rgba(5, 5, 20, 0.6)',
            borderRadius: '16px',
            border: '1px solid var(--cyber-cyan)',
            margin: '2rem 0',
            position: 'relative',
            overflow: 'hidden'
        }}>
            <div style={{
                position: 'absolute', top: 0, left: 0, width: '100%', height: '100%',
                backgroundImage: 'radial-gradient(circle at 50% 50%, rgba(0, 243, 255, 0.05) 0%, transparent 70%)',
                pointerEvents: 'none'
            }} />

            <div style={{ display: 'grid', gridTemplateColumns: '1fr 0.2fr 1.2fr', alignItems: 'center', gap: '1rem' }}>
                {/* Sensing Layer */}
                <div style={{ display: 'flex', flexDirection: 'column', gap: '1.5rem' }}>
                    <div style={{
                        padding: '1rem', background: 'rgba(0, 243, 255, 0.05)', border: '1px solid var(--cyber-cyan)',
                        borderRadius: '8px', textAlign: 'center'
                    }}>
                        <Activity size={20} color="var(--cyber-cyan)" />
                        <div style={{ fontSize: '0.8rem', marginTop: '0.5rem', fontFamily: 'Orbitron' }}>LiDAR Node</div>
                    </div>
                    <div style={{
                        padding: '1rem', background: 'rgba(0, 243, 255, 0.05)', border: '1px solid var(--cyber-cyan)',
                        borderRadius: '8px', textAlign: 'center'
                    }}>
                        <Activity size={20} color="var(--cyber-cyan)" />
                        <div style={{ fontSize: '0.8rem', marginTop: '0.5rem', fontFamily: 'Orbitron' }}>IMU Node</div>
                    </div>
                </div>

                {/* DDS Connection */}
                <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: '2rem' }}>
                    <Share2 size={24} color="var(--cyber-yellow)" />
                    <div style={{
                        width: '2px', height: '100px',
                        background: 'linear-gradient(to bottom, var(--cyber-cyan), var(--cyber-yellow), var(--cyber-purple))'
                    }} />
                </div>

                {/* Computation Layer */}
                <div style={{ display: 'flex', flexDirection: 'column', gap: '1rem' }}>
                    <div style={{
                        padding: '1.5rem', background: 'rgba(191, 0, 255, 0.1)', border: '1px solid var(--cyber-purple)',
                        borderRadius: '12px', borderLeft: '4px solid var(--cyber-yellow)'
                    }}>
                        <div style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', marginBottom: '0.5rem' }}>
                            <Zap size={16} color="var(--cyber-yellow)" />
                            <span style={{ fontSize: '0.9rem', fontWeight: 'bold', fontFamily: 'Orbitron' }}>DDS Middleware</span>
                        </div>
                        <p style={{ fontSize: '0.8rem', color: '#ccc', margin: 0 }}>
                            Decentralized discovery, peer-to-peer data transport, and QoS enforcement.
                        </p>
                    </div>

                    <div style={{ display: 'flex', gap: '1rem' }}>
                        <div style={{
                            flex: 1, padding: '0.75rem', background: 'rgba(255,255,255,0.05)',
                            border: '1px dashed #666', borderRadius: '4px', fontSize: '0.7rem', color: '#aaa'
                        }}>
                            Topics (Pub/Sub)
                        </div>
                        <div style={{
                            flex: 1, padding: '0.75rem', background: 'rgba(255,255,255,0.05)',
                            border: '1px dashed #666', borderRadius: '4px', fontSize: '0.7rem', color: '#aaa'
                        }}>
                            Actions (Goals)
                        </div>
                    </div>
                </div>
            </div>

            <div style={{
                marginTop: '1.5rem', textAlign: 'center', fontSize: '0.75rem',
                color: 'var(--cyber-cyan)', letterSpacing: '2px', fontFamily: 'Orbitron'
            }}>
                DATA STREAM DISCOVERED: 1024 packets/sec
            </div>
        </div>
    );
};

export default ROS2NervousSystemDiagram;
