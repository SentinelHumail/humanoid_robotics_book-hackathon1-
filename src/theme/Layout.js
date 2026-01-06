import React from 'react';
import Layout from '@theme-original/Layout';
import ParticleBackground from '../components/ParticleBackground';
import Chatbot from '../components/Chatbot';
import '../css/custom.css';

export default function CustomLayout(props) {
  return (
    <Layout {...props}>
      <ParticleBackground />
      {props.children}
      <Chatbot />
    </Layout>
  );
}
