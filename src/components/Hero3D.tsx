import React, { useRef, useState, useMemo } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { Sphere, Cylinder, Box, useCursor, Trail } from '@react-three/drei';
import * as THREE from 'three';

// --- Particle Effects ---
function Sparks({ count = 20 }) {
    const mesh = useRef<THREE.InstancedMesh>(null);
    const dummy = useMemo(() => new THREE.Object3D(), []);
    const particles = useMemo(() => {
        return new Array(count).fill(0).map(() => ({
            t: Math.random() * 100,
            factor: Math.random() * 0.5 + 0.5,
            speed: Math.random() * 0.01 + 0.005,
            xFactor: Math.random() * 2 - 1,
            yFactor: Math.random() * 2 - 1,
            zFactor: Math.random() * 2 - 1,
        }));
    }, [count]);

    useFrame((state, delta) => {
        if (!mesh.current) return;
        particles.forEach((particle, i) => {
            particle.t += particle.speed;
            const x = Math.sin(particle.t * 2) * 2 * particle.xFactor;
            const y = Math.cos(particle.t * 3) * 2 * particle.yFactor;
            const z = Math.sin(particle.t * 4) * 2 * particle.zFactor;

            dummy.position.set(x, y, z);
            const scale = (Math.sin(particle.t * 5) + 1) / 2 * 0.1;
            dummy.scale.set(scale, scale, scale);
            dummy.updateMatrix();
            mesh.current!.setMatrixAt(i, dummy.matrix);
        });
        mesh.current.instanceMatrix.needsUpdate = true;
    });

    return (
        <instancedMesh ref={mesh} args={[undefined, undefined, count]}>
            <sphereGeometry args={[0.2]} />
            <meshBasicMaterial color="#00f3ff" transparent opacity={0.6} />
        </instancedMesh>
    );
}

function RobotHead({ isHoveringButton }: { isHoveringButton: boolean }) {
    const meshRef = useRef<THREE.Group>(null);
    const [hovered, setHover] = useState(false);
    const [active, setActive] = useState(false);

    // Jump Animation State
    const timeRef = useRef(0);

    useCursor(hovered);

    useFrame((state) => {
        if (!meshRef.current) return;

        // 1. Precise Eye Tracking
        const targetX = (state.pointer.x * Math.PI) / 4;
        const targetY = (-state.pointer.y * Math.PI) / 4;

        // Add Button Lean
        const leanX = isHoveringButton ? targetX * 1.5 : targetX;
        const leanY = isHoveringButton ? targetY * 1.5 : targetY;

        meshRef.current.rotation.y = THREE.MathUtils.lerp(meshRef.current.rotation.y, leanX, 0.1);
        meshRef.current.rotation.x = THREE.MathUtils.lerp(meshRef.current.rotation.x, leanY, 0.1);

        // 2. Breathing / Idling
        const breath = Math.sin(state.clock.elapsedTime * 2) * 0.05;
        meshRef.current.position.y = breath;

        // 3. Jump Animation
        if (active) {
            timeRef.current += 0.15;
            const jumpHeight = Math.sin(timeRef.current) * 2;
            if (jumpHeight < 0) {
                setActive(false);
                timeRef.current = 0;
                meshRef.current.position.y = breath; // Reset to idle
            } else {
                meshRef.current.position.y = jumpHeight;
            }
        }
    });

    return (
        <group
            ref={meshRef}
            onPointerOver={() => setHover(true)}
            onPointerOut={() => setHover(false)}
            onClick={() => setActive(true)}
        >
            {/* Robot Head */}
            <Box args={[1.2, 1.4, 1.2]} position={[0, 0, 0]}>
                <meshStandardMaterial
                    color="#1a1a1a"
                    roughness={0.2}
                    metalness={0.9}
                />
            </Box>

            {/* Purple Accent Stripes */}
            <Box args={[1.3, 0.1, 1.3]} position={[0, 0.5, 0]}>
                <meshStandardMaterial
                    color="#bf00ff"
                    emissive="#bf00ff"
                    emissiveIntensity={0.3}
                    metalness={0.9}
                    roughness={0.1}
                />
            </Box>
            <Box args={[1.3, 0.1, 1.3]} position={[0, -0.5, 0]}>
                <meshStandardMaterial
                    color="#bf00ff"
                    emissive="#bf00ff"
                    emissiveIntensity={0.3}
                    metalness={0.9}
                    roughness={0.1}
                />
            </Box>

            {/* Ninja Tech Mask */}
            <Box args={[1.1, 0.6, 0.15]} position={[0, -0.1, 0.65]}>
                <meshStandardMaterial
                    color="#0a0a0a"
                    metalness={0.8}
                    roughness={0.2}
                />
            </Box>
            {/* Mask Glow Details */}
            <Box args={[1.0, 0.05, 0.16]} position={[0, -0.35, 0.66]}>
                <meshStandardMaterial
                    color="#bf00ff"
                    emissive="#bf00ff"
                    emissiveIntensity={0.5}
                />
            </Box>

            {/* Face Screen */}
            <Box args={[1.0, 0.8, 0.1]} position={[0, 0, 0.6]}>
                <meshStandardMaterial color="#000" />
            </Box>

            {/* Eyes */}
            <Sphere args={[0.15, 32, 32]} position={[-0.25, 0, 0.65]}>
                <meshStandardMaterial
                    color="#fcee0a"
                    emissive="#fcee0a"
                    emissiveIntensity={active ? 5 : 3}
                />
            </Sphere>
            <Sphere args={[0.15, 32, 32]} position={[0.25, 0, 0.65]}>
                <meshStandardMaterial
                    color="#fcee0a"
                    emissive="#fcee0a"
                    emissiveIntensity={active ? 5 : 3}
                />
            </Sphere>

            
            {/* Antennas */}
            <Cylinder args={[0.05, 0.05, 0.5]} position={[-0.6, 0.8, 0]} rotation={[0, 0, 0.5]}>
                <meshStandardMaterial color="#bf00ff" metalness={0.8} emissive="#bf00ff" emissiveIntensity={0.5} />
            </Cylinder>
            <Cylinder args={[0.05, 0.05, 0.5]} position={[0.6, 0.8, 0]} rotation={[0, 0, -0.5]}>
                <meshStandardMaterial color="#bf00ff" metalness={0.8} emissive="#bf00ff" emissiveIntensity={0.5} />
            </Cylinder>

            {/* Neck/Stand connection */}
            <Cylinder args={[0.4, 0.6, 0.5]} position={[0, -1, 0]}>
                <meshStandardMaterial color="#333" />
            </Cylinder>
        </group>
    );
}

interface Hero3DProps {
    isHoveringButton: boolean;
}

export default function Hero3D({ isHoveringButton }: Hero3DProps) {
    return (
        <div style={{ width: '100%', height: '500px' }}>
            <Canvas camera={{ position: [0, 0, 6], fov: 45 }}>
                <ambientLight intensity={0.5} />
                <pointLight position={[10, 10, 10]} intensity={1} color="#00f3ff" />
                <spotLight position={[-10, 10, 10]} angle={0.3} penumbra={1} intensity={5} color="#bf00ff" />

                <RobotHead isHoveringButton={isHoveringButton} />
                <Sparks count={30} />
            </Canvas>
        </div>
    );
}
