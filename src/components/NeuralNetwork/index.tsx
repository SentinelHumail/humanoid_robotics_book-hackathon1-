import React, { useEffect, useRef } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

interface NetworkNode {
  x: number;
  y: number;
  vx: number;
  vy: number;
  radius: number;
}

interface Connection {
  nodeA: number;
  nodeB: number;
  opacity: number;
}

const NeuralNetwork = () => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const animationRef = useRef<number>(null!);
  const nodesRef = useRef<NetworkNode[]>([]);
  const connectionsRef = useRef<Connection[]>([]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const resizeCanvas = () => {
      const parent = canvas.parentElement;
      if (parent) {
        canvas.width = parent.clientWidth;
        canvas.height = parent.clientHeight;
        initNodes();
        initConnections();
      }
    };

    const initNodes = () => {
      const nodes: NetworkNode[] = [];
      const layers = [4, 6, 8, 6, 4];
      const layerGap = canvas.width / (layers.length + 1);
      const maxY = canvas.height;

      let nodeIndex = 0;
      layers.forEach((count, layerIndex) => {
        const x = layerGap * (layerIndex + 1);
        const yGap = maxY / (count + 1);
        for (let i = 0; i < count; i++) {
          nodes.push({
            x: x + (Math.random() - 0.5) * 30,
            y: yGap * (i + 1) + (Math.random() - 0.5) * 20,
            vx: (Math.random() - 0.5) * 0.5,
            vy: (Math.random() - 0.5) * 0.5,
            radius: 3 + Math.random() * 3 + (layerIndex === 2 ? 2 : 0),
          });
          nodeIndex++;
        }
      });
      nodesRef.current = nodes;
    };

    const initConnections = () => {
      const connections: Connection[] = [];
      const layers = [4, 6, 8, 6, 4];
      let nodeIndex = 0;
      let layerStartIndices: number[] = [];

      layers.forEach((count, i) => {
        layerStartIndices.push(nodeIndex);
        nodeIndex += count;
      });

      for (let l = 0; l < layers.length - 1; l++) {
        for (let i = 0; i < layers[l]; i++) {
          for (let j = 0; j < layers[l + 1]; j++) {
            if (Math.random() > 0.3) {
              connections.push({
                nodeA: layerStartIndices[l] + i,
                nodeB: layerStartIndices[l + 1] + j,
                opacity: 0.1 + Math.random() * 0.2,
              });
            }
          }
        }
      }
      connectionsRef.current = connections;
    };

    const animate = () => {
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // Update nodes
      nodesRef.current.forEach((node) => {
        node.x += node.vx;
        node.y += node.vy;

        if (node.x < 50 || node.x > canvas.width - 50) node.vx *= -1;
        if (node.y < 50 || node.y > canvas.height - 50) node.vy *= -1;
      });

      // Draw connections
      connectionsRef.current.forEach((conn) => {
        const nodeA = nodesRef.current[conn.nodeA];
        const nodeB = nodesRef.current[conn.nodeB];
        if (!nodeA || !nodeB) return;

        const gradient = ctx.createLinearGradient(nodeA.x, nodeA.y, nodeB.x, nodeB.y);
        gradient.addColorStop(0, `rgba(0, 212, 255, ${conn.opacity})`);
        gradient.addColorStop(1, `rgba(99, 102, 241, ${conn.opacity})`);

        ctx.beginPath();
        ctx.moveTo(nodeA.x, nodeA.y);
        ctx.lineTo(nodeB.x, nodeB.y);
        ctx.strokeStyle = gradient;
        ctx.lineWidth = 1;
        ctx.stroke();
      });

      // Draw nodes
      nodesRef.current.forEach((node, index) => {
        const isCenter = node.radius > 5;

        // Glow effect
        if (isCenter) {
          const glow = ctx.createRadialGradient(
            node.x, node.y, 0,
            node.x, node.y, node.radius * 4
          );
          glow.addColorStop(0, 'rgba(0, 212, 255, 0.4)');
          glow.addColorStop(0.5, 'rgba(99, 102, 241, 0.2)');
          glow.addColorStop(1, 'transparent');

          ctx.beginPath();
          ctx.arc(node.x, node.y, node.radius * 4, 0, Math.PI * 2);
          ctx.fillStyle = glow;
          ctx.fill();
        }

        // Node
        ctx.beginPath();
        ctx.arc(node.x, node.y, node.radius, 0, Math.PI * 2);

        const nodeGradient = ctx.createRadialGradient(
          node.x - node.radius / 3, node.y - node.radius / 3, 0,
          node.x, node.y, node.radius
        );

        if (isCenter) {
          nodeGradient.addColorStop(0, '#66EBFF');
          nodeGradient.addColorStop(0.5, '#00D4FF');
          nodeGradient.addColorStop(1, '#6366F1');
        } else {
          nodeGradient.addColorStop(0, '#00D4FF');
          nodeGradient.addColorStop(1, '#00668C');
        }

        ctx.fillStyle = nodeGradient;
        ctx.fill();
      });

      animationRef.current = requestAnimationFrame(animate);
    };

    resizeCanvas();
    window.addEventListener('resize', resizeCanvas);
    animate();

    return () => {
      window.removeEventListener('resize', resizeCanvas);
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
    };
  }, []);

  return (
    <canvas
      ref={canvasRef}
      className={styles.canvas}
    />
  );
};

export default NeuralNetwork;
