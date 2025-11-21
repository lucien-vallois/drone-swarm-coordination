#!/usr/bin/env python3
"""
Script para gerar demo GIF da simulação de drone swarm.
Gera frames da simulação e monta um GIF animado.
"""

import argparse
import math
import os
import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from PIL import Image

# Adicionar o diretório raiz ao path
sys.path.insert(0, str(Path(__file__).parent.parent))

from backend.coordinator import Formation, SwarmConfig, SwarmCoordinator


def generate_simulation_frames(num_drones=10, duration=10, fps=2, output_dir="temp_frames"):
    """
    Gera frames da simulação de drone swarm.
    
    Args:
        num_drones: Número de drones na simulação
        duration: Duração da simulação em segundos
        fps: Frames por segundo para captura
        output_dir: Diretório para salvar frames temporários
    """
    # Criar diretório de frames
    os.makedirs(output_dir, exist_ok=True)
    
    # Configurar simulação
    config = SwarmConfig(
        num_drones=num_drones,
        formation=Formation.CIRCLE,
        separation_distance=8.0,
        max_velocity=3.0
    )
    
    coordinator = SwarmCoordinator(config)
    
    # Adicionar alguns obstáculos
    coordinator.add_obstacle([30, 30, 10], 5)
    coordinator.add_obstacle([70, 70, 15], 4)
    coordinator.add_obstacle([50, 20, 12], 3)
    
    # Parâmetros de simulação
    dt = 0.1
    total_steps = int(duration / dt)
    capture_interval = int(1.0 / (fps * dt))
    
    frames = []
    step_count = 0
    
    print(f"Gerando {total_steps} steps de simulação...")
    
    # Lista de formações para demonstrar
    formations = [
        (Formation.CIRCLE, "Circle Formation"),
        (Formation.LINE, "Line Formation"),
        (Formation.GRID, "Grid Formation"),
        (Formation.V_FORMATION, "V Formation"),
    ]
    formation_index = 0
    formation_change_interval = total_steps // len(formations)
    
    for step in range(total_steps):
        # Mudar formação periodicamente
        if step > 0 and step % formation_change_interval == 0:
            if formation_index < len(formations) - 1:
                formation_index += 1
                coordinator.set_formation(formations[formation_index][0])
                print(f"Mudando para formação: {formations[formation_index][1]}")
        
        # Executar step da simulação
        state = coordinator.step(dt)
        
        # Capturar frame periodicamente
        if step % capture_interval == 0:
            frame_data = {
                'step': step,
                'time': step * dt,
                'state': state,
                'formation': formations[formation_index][1]
            }
            frames.append(frame_data)
            step_count += 1
    
    print(f"Capturados {len(frames)} frames")
    return frames, coordinator


def create_visualization_frame(frame_data, frame_num, output_path, bounds=(100, 100, 50)):
    """
    Cria uma visualização 2D/3D de um frame da simulação.
    
    Args:
        frame_data: Dados do frame
        frame_num: Número do frame
        output_path: Caminho para salvar a imagem
        bounds: Limites do ambiente (x, y, z)
    """
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    state = frame_data['state']
    drones = state.get('drones', {})
    obstacles = state.get('obstacles', [])
    leader_id = state.get('leader_id')
    
    # Plotar obstáculos
    for obs in obstacles:
        pos = obs['position']
        radius = obs['radius']
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = pos[0] + radius * np.outer(np.cos(u), np.sin(v))
        y = pos[1] + radius * np.outer(np.sin(u), np.sin(v))
        z = pos[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
        ax.plot_surface(x, y, z, alpha=0.3, color='orange')
    
    # Plotar drones
    for drone_id, drone in drones.items():
        pos = drone['position']
        is_leader = drone_id == leader_id
        
        # Cor diferente para o líder
        color = 'cyan' if is_leader else 'green'
        size = 150 if is_leader else 100
        
        ax.scatter(pos[0], pos[1], pos[2], c=color, s=size, 
                  marker='o', edgecolors='black', linewidths=1.5)
        
        # Adicionar label
        ax.text(pos[0], pos[1], pos[2] + 2, f'D{drone_id}', 
               fontsize=8, ha='center')
        
        # Mostrar velocidade como seta
        vel = drone.get('velocity', [0, 0, 0])
        if np.linalg.norm(vel) > 0.1:
            ax.quiver(pos[0], pos[1], pos[2],
                     vel[0], vel[1], vel[2],
                     length=3, normalize=True, color=color, alpha=0.6)
    
    # Configurar eixos
    ax.set_xlim(0, bounds[0])
    ax.set_ylim(0, bounds[1])
    ax.set_zlim(0, bounds[2])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f"Drone Swarm Simulation - {frame_data['formation']}\n"
                f"Time: {frame_data['time']:.1f}s | Frame: {frame_num} | Leader: D{leader_id if leader_id else 'None'}")
    
    # Adicionar grid
    ax.grid(True, alpha=0.3)
    
    # Salvar frame
    plt.tight_layout()
    plt.savefig(output_path, dpi=100, bbox_inches='tight')
    plt.close(fig)


def frames_to_gif(frame_dir, output_gif, duration=0.5):
    """
    Converte frames PNG em um GIF animado.
    
    Args:
        frame_dir: Diretório com frames PNG
        output_gif: Caminho do arquivo GIF de saída
        duration: Duração de cada frame em segundos
    """
    frames = []
    frame_files = sorted([f for f in os.listdir(frame_dir) if f.endswith('.png')])
    
    print(f"Convertendo {len(frame_files)} frames em GIF...")
    
    for frame_file in frame_files:
        frame_path = os.path.join(frame_dir, frame_file)
        img = Image.open(frame_path)
        frames.append(img)
    
    if frames:
        # Salvar como GIF animado
        frames[0].save(
            output_gif,
            save_all=True,
            append_images=frames[1:],
            duration=int(duration * 1000),  # Converter para milissegundos
            loop=0
        )
        print(f"GIF salvo em: {output_gif}")
    else:
        print("Nenhum frame encontrado!")


def main():
    parser = argparse.ArgumentParser(description='Gerar demo GIF da simulação de drone swarm')
    parser.add_argument('--num-drones', type=int, default=10, help='Número de drones')
    parser.add_argument('--duration', type=float, default=15, help='Duração da simulação em segundos')
    parser.add_argument('--fps', type=float, default=2, help='Frames por segundo para captura')
    parser.add_argument('--output', type=str, default='assets/demo.gif', help='Caminho do arquivo GIF de saída')
    parser.add_argument('--temp-dir', type=str, default='temp_frames', help='Diretório temporário para frames')
    parser.add_argument('--keep-frames', action='store_true', help='Manter frames temporários após gerar GIF')
    
    args = parser.parse_args()
    
    # Garantir que o diretório de saída existe
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    print("=" * 60)
    print("Gerador de Demo GIF - Drone Swarm Coordination")
    print("=" * 60)
    
    # Gerar frames da simulação
    print("\n[1/3] Gerando frames da simulação...")
    frames, coordinator = generate_simulation_frames(
        num_drones=args.num_drones,
        duration=args.duration,
        fps=args.fps,
        output_dir=args.temp_dir
    )
    
    # Criar visualizações
    print(f"\n[2/3] Criando visualizações ({len(frames)} frames)...")
    for i, frame_data in enumerate(frames):
        frame_path = os.path.join(args.temp_dir, f"frame_{i:04d}.png")
        create_visualization_frame(frame_data, i, frame_path)
        if (i + 1) % 10 == 0:
            print(f"  Processados {i + 1}/{len(frames)} frames...")
    
    # Converter para GIF
    print(f"\n[3/3] Convertendo frames em GIF...")
    frames_to_gif(args.temp_dir, args.output, duration=1.0 / args.fps)
    
    # Limpar frames temporários
    if not args.keep_frames:
        print(f"\nLimpando frames temporários...")
        import shutil
        shutil.rmtree(args.temp_dir)
        print("Limpeza concluída!")
    
    print("\n" + "=" * 60)
    print(f"Demo GIF gerado com sucesso: {args.output}")
    print("=" * 60)


if __name__ == "__main__":
    main()







