// Global variables
let scene, camera, renderer, controls;
let drones = new Map();
let obstacles = [];
let targetPositions = new Map();
let websocket = null;
let animationId = null;
let currentState = null;
let droneLabels = new Map();
let droneTrails = new Map(); // Trilhas de movimento
let dronePaths = new Map(); // Caminhos planejados
let animationTime = 0;

// Colors
const DRONE_COLOR = 0x00ff00;
const LEADER_COLOR = 0x00ffff; // Ciano para líder
const TARGET_COLOR = 0xff0000;
const OBSTACLE_COLOR = 0xffa500;
const PATH_COLOR = 0x0000ff;
const TRAIL_COLOR = 0x00ff88;

// Initialize Three.js scene
function init() {
    // Create scene com gradiente de céu
    scene = new THREE.Scene();
    // Gradiente de céu mais realista
    const skyColorTop = new THREE.Color(0x87CEEB);
    const skyColorBottom = new THREE.Color(0xE0F6FF);
    scene.background = skyColorTop;
    scene.fog = new THREE.FogExp2(0x87CEEB, 0.002); // Névoa suave para profundidade

    // Create camera
    camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(60, 60, 60);
    camera.lookAt(50, 25, 25);

    // Create renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.setPixelRatio(window.devicePixelRatio);
    document.getElementById('container').appendChild(renderer.domElement);

    // Create controls
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;

    // Add lighting
    addLighting();

    // Add ground
    addGround();

    // Add coordinate system
    addCoordinateSystem();

    // Connect to WebSocket
    connectWebSocket();

    // Start animation loop
    animate();
}

function addLighting() {
    // Ambient light
    const ambientLight = new THREE.AmbientLight(0x404040, 0.7);
    scene.add(ambientLight);

    // Directional light (sun)
    const directionalLight = new THREE.DirectionalLight(0xffffff, 1.0);
    directionalLight.position.set(50, 80, 50);
    directionalLight.castShadow = true;
    directionalLight.shadow.mapSize.width = 2048;
    directionalLight.shadow.mapSize.height = 2048;
    directionalLight.shadow.camera.near = 0.5;
    directionalLight.shadow.camera.far = 500;
    directionalLight.shadow.camera.left = -100;
    directionalLight.shadow.camera.right = 100;
    directionalLight.shadow.camera.top = 100;
    directionalLight.shadow.camera.bottom = -100;
    directionalLight.shadow.bias = -0.0001;
    scene.add(directionalLight);

    // Point light para melhor iluminação dos drones
    const pointLight = new THREE.PointLight(0xffffff, 0.5, 200);
    pointLight.position.set(50, 50, 50);
    scene.add(pointLight);

    // Hemispheric light para iluminação mais natural
    const hemisphereLight = new THREE.HemisphereLight(0x87CEEB, 0x90EE90, 0.4);
    scene.add(hemisphereLight);
}

function addGround() {
    // Ground plane com textura melhorada
    const groundGeometry = new THREE.PlaneGeometry(200, 200, 20, 20);
    const groundMaterial = new THREE.MeshLambertMaterial({
        color: 0x7cb342,
        transparent: true,
        opacity: 0.9
    });
    const ground = new THREE.Mesh(groundGeometry, groundMaterial);
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = 0;
    ground.receiveShadow = true;
    scene.add(ground);

    // Grid helper mais visível
    const gridHelper = new THREE.GridHelper(200, 20, 0x333333, 0x666666);
    gridHelper.position.y = 0.01;
    scene.add(gridHelper);

    // Adicionar algumas nuvens decorativas no fundo
    for (let i = 0; i < 5; i++) {
        const cloudGeometry = new THREE.SphereGeometry(8 + Math.random() * 4, 8, 8);
        const cloudMaterial = new THREE.MeshBasicMaterial({
            color: 0xffffff,
            transparent: true,
            opacity: 0.3
        });
        const cloud = new THREE.Mesh(cloudGeometry, cloudMaterial);
        cloud.position.set(
            (Math.random() - 0.5) * 150,
            40 + Math.random() * 20,
            (Math.random() - 0.5) * 150
        );
        scene.add(cloud);
    }
}

function addCoordinateSystem() {
    // X axis (red)
    const xAxis = new THREE.ArrowHelper(
        new THREE.Vector3(1, 0, 0),
        new THREE.Vector3(0, 0, 0),
        10, 0xff0000
    );
    scene.add(xAxis);

    // Y axis (green)
    const yAxis = new THREE.ArrowHelper(
        new THREE.Vector3(0, 1, 0),
        new THREE.Vector3(0, 0, 0),
        10, 0x00ff00
    );
    scene.add(yAxis);

    // Z axis (blue)
    const zAxis = new THREE.ArrowHelper(
        new THREE.Vector3(0, 0, 1),
        new THREE.Vector3(0, 0, 0),
        10, 0x0000ff
    );
    scene.add(zAxis);
}

function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const host = window.location.host || 'localhost:8765';
    const wsUrl = `${protocol}//${host}`;

    websocket = new WebSocket(wsUrl);

    websocket.onopen = function(event) {
        updateStatus('connection-status', 'Connected', 'green');
        updateStatus('simulation-status', 'Ready', 'blue');

        // Request initial state
        sendMessage({
            type: 'request',
            request: 'state'
        });

        sendMessage({
            type: 'request',
            request: 'config'
        });
    };

    websocket.onmessage = function(event) {
        try {
            const data = JSON.parse(event.data);
            handleMessage(data);
        } catch (error) {
            console.error('Failed to parse WebSocket message:', error);
        }
    };

    websocket.onclose = function(event) {
        updateStatus('connection-status', 'Disconnected', 'red');
        updateStatus('simulation-status', 'Offline', 'red');

        // Attempt to reconnect after 5 seconds
        setTimeout(connectWebSocket, 5000);
    };

    websocket.onerror = function(error) {
        console.error('WebSocket error:', error);
        updateStatus('connection-status', 'Error', 'red');
    };
}

function handleMessage(data) {
    if (data.type === 'state' && data.data) {
        currentState = data.data;
        updateVisualization();
        updateStats();
    } else if (data.type === 'config' && data.data) {
        updateConfig(data.data);
    } else if (data.status) {
        updateStatus('simulation-status', data.status, 'green');
    } else if (data.error) {
        console.error('Server error:', data.error);
        updateStatus('simulation-status', 'Error: ' + data.error, 'red');
    }
}

function updateVisualization() {
    if (!currentState) return;

    // Update drones
    updateDrones();

    // Update obstacles
    updateObstacles();

    // Update target positions
    updateTargets();
}

function updateDrones() {
    const coordinatorDrones = currentState.coordinator.drones || {};
    const leaderId = currentState.coordinator?.leader_id;

    // Remove drones that no longer exist
    for (const [droneId, droneMesh] of drones) {
        if (!(droneId in coordinatorDrones)) {
            scene.remove(droneMesh);
            drones.delete(droneId);

            // Remove trail
            const trail = droneTrails.get(droneId);
            if (trail) {
                scene.remove(trail);
                droneTrails.delete(droneId);
            }

            // Remove path
            const path = dronePaths.get(droneId);
            if (path) {
                scene.remove(path);
                dronePaths.delete(droneId);
            }

            // Remove label
            const label = droneLabels.get(droneId);
            if (label) {
                document.body.removeChild(label);
                droneLabels.delete(droneId);
            }
        }
    }

    // Update or create drones
    for (const [droneId, droneData] of Object.entries(coordinatorDrones)) {
        const id = parseInt(droneId);
        const position = droneData.position;
        const isActive = droneData.is_active;
        const isLeader = leaderId !== null && leaderId !== undefined && id === leaderId;

        let droneMesh = drones.get(id);

        if (!droneMesh) {
            // Create drone group
            droneMesh = new THREE.Group();

            // Main body - formato mais realista de drone
            const bodyGeometry = new THREE.BoxGeometry(0.8, 0.15, 0.8);
            const bodyMaterial = new THREE.MeshPhongMaterial({
                color: isLeader ? LEADER_COLOR : DRONE_COLOR,
                shininess: 150,
                specular: 0x222222
            });
            const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
            body.castShadow = true;
            body.receiveShadow = true;
            droneMesh.add(body);

            // Central core
            const coreGeometry = new THREE.CylinderGeometry(0.2, 0.2, 0.2, 16);
            const coreMaterial = new THREE.MeshPhongMaterial({
                color: 0x222222,
                shininess: 200
            });
            const core = new THREE.Mesh(coreGeometry, coreMaterial);
            core.position.y = 0.1;
            core.castShadow = true;
            droneMesh.add(core);

            // Add propellers com suportes
            const propPositions = [
                { x: 0.5, z: 0.5 },
                { x: -0.5, z: 0.5 },
                { x: -0.5, z: -0.5 },
                { x: 0.5, z: -0.5 }
            ];

            for (let i = 0; i < 4; i++) {
                const pos = propPositions[i];
                
                // Suporte da hélice
                const armGeometry = new THREE.CylinderGeometry(0.03, 0.03, 0.5, 8);
                const armMaterial = new THREE.MeshPhongMaterial({ color: 0x444444 });
                const arm = new THREE.Mesh(armGeometry, armMaterial);
                arm.position.set(pos.x * 0.4, 0.25, pos.z * 0.4);
                arm.rotation.z = Math.atan2(pos.z, pos.x);
                arm.castShadow = true;
                droneMesh.add(arm);

                // Hélice rotativa
                const propGeometry = new THREE.CylinderGeometry(0.6, 0.6, 0.02, 16);
                const propMaterial = new THREE.MeshPhongMaterial({ 
                    color: 0x111111,
                    transparent: true,
                    opacity: 0.8
                });
                const propeller = new THREE.Mesh(propGeometry, propMaterial);
                propeller.position.set(pos.x * 0.4, 0.5, pos.z * 0.4);
                propeller.rotation.x = Math.PI / 2;
                propeller.userData.isPropeller = true;
                propeller.userData.propellerIndex = i;
                droneMesh.add(propeller);
            }

            // LED indicador
            const ledGeometry = new THREE.SphereGeometry(0.05, 8, 8);
            const ledMaterial = new THREE.MeshBasicMaterial({
                color: isLeader ? LEADER_COLOR : DRONE_COLOR,
                emissive: isLeader ? LEADER_COLOR : DRONE_COLOR,
                emissiveIntensity: 0.8
            });
            const led = new THREE.Mesh(ledGeometry, ledMaterial);
            led.position.y = 0.15;
            droneMesh.add(led);

            droneMesh.castShadow = true;
            droneMesh.receiveShadow = true;
            scene.add(droneMesh);
            drones.set(id, droneMesh);

            // Initialize trail
            const trailGeometry = new THREE.BufferGeometry();
            const trailMaterial = new THREE.LineBasicMaterial({
                color: TRAIL_COLOR,
                transparent: true,
                opacity: 0.6,
                linewidth: 2
            });
            const trail = new THREE.Line(trailGeometry, trailMaterial);
            scene.add(trail);
            droneTrails.set(id, { line: trail, positions: [] });

            // Add label
            const label = document.createElement('div');
            label.className = 'drone-label';
            label.textContent = isLeader ? `Líder ${id}` : `Drone ${id}`;
            document.body.appendChild(label);
            droneLabels.set(id, label);
        }

        // Update position
        const pos3d = new THREE.Vector3(position[0], position[2], position[1]);
        droneMesh.position.copy(pos3d);

        // Update color based on leader status
        const body = droneMesh.children[0];
        if (body && body.material) {
            body.material.color.setHex(isLeader ? LEADER_COLOR : (isActive ? DRONE_COLOR : 0x666666));
        }

        // Update LED
        const led = droneMesh.children.find(child => child.material && child.material.emissive);
        if (led && led.material) {
            const ledColor = isLeader ? LEADER_COLOR : (isActive ? DRONE_COLOR : 0x666666);
            led.material.color.setHex(ledColor);
            led.material.emissive.setHex(ledColor);
        }

        // Update trail
        updateDroneTrail(id, pos3d);

        // Update label
        const label = droneLabels.get(id);
        if (label) {
            label.textContent = isLeader ? `Líder ${id}` : `Drone ${id}`;
        }
        updateDroneLabel(id, position);

        // Rotate propellers
        const propellers = droneMesh.children.filter(child => child.userData.isPropeller);
        propellers.forEach((prop, index) => {
            if (isActive) {
                prop.rotation.z += 0.5 + index * 0.1; // Rotação animada
            }
        });
    }
}

function updateDroneTrail(droneId, position) {
    const trailData = droneTrails.get(droneId);
    if (!trailData) return;

    // Adicionar nova posição
    trailData.positions.push(position.clone());

    // Limitar tamanho da trilha (últimas 50 posições)
    const maxTrailLength = 50;
    if (trailData.positions.length > maxTrailLength) {
        trailData.positions.shift();
    }

    // Atualizar geometria
    if (trailData.positions.length > 1) {
        const positions = new Float32Array(trailData.positions.length * 3);
        trailData.positions.forEach((pos, i) => {
            positions[i * 3] = pos.x;
            positions[i * 3 + 1] = pos.y;
            positions[i * 3 + 2] = pos.z;
        });
        trailData.line.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        trailData.line.geometry.setDrawRange(0, trailData.positions.length);
    }
}

function updateDroneLabel(droneId, position) {
    const label = droneLabels.get(droneId);
    if (!label) return;

    const vector = new THREE.Vector3(position[0], position[2], position[1]);
    vector.project(camera);

    const x = (vector.x * 0.5 + 0.5) * window.innerWidth;
    const y = (-vector.y * 0.5 + 0.5) * window.innerHeight;

    label.style.left = `${x}px`;
    label.style.top = `${y - 30}px`; // Offset above drone
}

function updateObstacles() {
    const environmentObstacles = currentState.environment?.obstacles || [];

    // Remove old obstacles
    obstacles.forEach(obstacle => scene.remove(obstacle));
    obstacles = [];

    // Add new obstacles com visual melhorado
    environmentObstacles.forEach((obs, index) => {
        const obstacleGroup = new THREE.Group();

        // Esfera principal
        const geometry = new THREE.SphereGeometry(obs.radius, 32, 32);
        const material = new THREE.MeshPhongMaterial({
            color: OBSTACLE_COLOR,
            transparent: true,
            opacity: 0.6,
            emissive: OBSTACLE_COLOR,
            emissiveIntensity: 0.2
        });
        const obstacleMesh = new THREE.Mesh(geometry, material);
        obstacleMesh.position.set(obs.position[0], obs.position[2], obs.position[1]);
        obstacleMesh.castShadow = true;
        obstacleMesh.receiveShadow = true;
        obstacleGroup.add(obstacleMesh);

        // Anel de aviso
        const warningRingGeometry = new THREE.TorusGeometry(obs.radius * 1.2, 0.1, 8, 16);
        const warningRingMaterial = new THREE.MeshBasicMaterial({
            color: 0xff4444,
            transparent: true,
            opacity: 0.4
        });
        const warningRing = new THREE.Mesh(warningRingGeometry, warningRingMaterial);
        warningRing.position.set(obs.position[0], obs.position[2], obs.position[1]);
        warningRing.rotation.x = Math.PI / 2;
        warningRing.userData.isWarningRing = true;
        obstacleGroup.add(warningRing);

        scene.add(obstacleGroup);
        obstacles.push(obstacleGroup);
    });
}

function updateObstacleAnimations() {
    // Animar anéis de aviso dos obstáculos
    obstacles.forEach(obstacleGroup => {
        const ring = obstacleGroup.children.find(child => child.userData.isWarningRing);
        if (ring) {
            ring.rotation.z += 0.01;
            const pulse = 1 + Math.sin(animationTime * 2) * 0.1;
            ring.scale.set(pulse, pulse, 1);
        }
    });
}

function updateTargets() {
    const targets = currentState.coordinator?.target_positions || {};

    // Remove old targets
    for (const [droneId, targetMesh] of targetPositions) {
        scene.remove(targetMesh);
        targetPositions.delete(droneId);
    }

    // Add new targets com animação pulsante
    for (const [droneId, targetPos] of Object.entries(targets)) {
        const id = parseInt(droneId);
        
        // Criar grupo para target com múltiplos elementos
        const targetGroup = new THREE.Group();
        
        // Esfera principal
        const geometry = new THREE.SphereGeometry(0.3, 16, 16);
        const material = new THREE.MeshPhongMaterial({
            color: TARGET_COLOR,
            transparent: true,
            opacity: 0.7,
            emissive: TARGET_COLOR,
            emissiveIntensity: 0.3
        });
        const targetMesh = new THREE.Mesh(geometry, material);
        targetMesh.position.set(targetPos[0], targetPos[2], targetPos[1]);
        targetMesh.userData.isTarget = true;
        targetGroup.add(targetMesh);

        // Anel pulsante
        const ringGeometry = new THREE.TorusGeometry(0.4, 0.05, 8, 16);
        const ringMaterial = new THREE.MeshBasicMaterial({
            color: TARGET_COLOR,
            transparent: true,
            opacity: 0.5
        });
        const ring = new THREE.Mesh(ringGeometry, ringMaterial);
        ring.position.set(targetPos[0], targetPos[2], targetPos[1]);
        ring.rotation.x = Math.PI / 2;
        ring.userData.isRing = true;
        targetGroup.add(ring);

        // Linha conectando drone ao target (se drone existir)
        const droneMesh = drones.get(id);
        if (droneMesh) {
            const lineGeometry = new THREE.BufferGeometry();
            const lineMaterial = new THREE.LineDashedMaterial({
                color: TARGET_COLOR,
                transparent: true,
                opacity: 0.3,
                dashSize: 0.5,
                gapSize: 0.3
            });
            const line = new THREE.Line(lineGeometry, lineMaterial);
            line.computeLineDistances();
            line.userData.isTargetLine = true;
            targetGroup.add(line);
        }

        scene.add(targetGroup);
        targetPositions.set(id, targetGroup);
    }
}

function updateTargetAnimations() {
    // Animar targets
    for (const [droneId, targetGroup] of targetPositions) {
        const target = targetGroup.children.find(child => child.userData.isTarget);
        const ring = targetGroup.children.find(child => child.userData.isRing);
        const line = targetGroup.children.find(child => child.userData.isTargetLine);

        if (target) {
            // Pulsação suave
            const scale = 1 + Math.sin(animationTime * 2) * 0.1;
            target.scale.set(scale, scale, scale);
        }

        if (ring) {
            // Rotação do anel
            ring.rotation.z += 0.02;
            const ringScale = 1 + Math.sin(animationTime * 3) * 0.2;
            ring.scale.set(ringScale, ringScale, 1);
        }

        // Atualizar linha para target
        if (line && drones.has(droneId)) {
            const droneMesh = drones.get(droneId);
            const targetPos = targetGroup.position;
            const dronePos = droneMesh.position;

            const positions = new Float32Array([
                dronePos.x, dronePos.y, dronePos.z,
                targetPos.x, targetPos.y, targetPos.z
            ]);
            line.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
            line.computeLineDistances();
        }
    }
}

function updateStats() {
    if (!currentState) return;

    document.getElementById('drone-count').textContent = Object.keys(currentState.coordinator?.drones || {}).length;
    document.getElementById('fps').textContent = currentState.fps?.toFixed(1) || '0';
    document.getElementById('frame').textContent = currentState.frame || '0';
    document.getElementById('leader').textContent = currentState.coordinator?.leader_id ?? 'None';
}

function updateConfig(config) {
    // Update UI with config
    console.log('Config received:', config);
}

function sendMessage(message) {
    if (websocket && websocket.readyState === WebSocket.OPEN) {
        websocket.send(JSON.stringify(message));
    }
}

function updateStatus(elementId, text, color) {
    const element = document.getElementById(elementId);
    if (element) {
        element.textContent = text;
        element.style.color = color;
    }
}

function animate() {
    animationId = requestAnimationFrame(animate);
    animationTime += 0.016; // Aproximadamente 60 FPS

    // Update controls
    controls.update();

    // Update drone labels
    if (currentState?.coordinator?.drones) {
        for (const [droneId, droneData] of Object.entries(currentState.coordinator.drones)) {
            updateDroneLabel(parseInt(droneId), droneData.position);
        }
    }

    // Update target animations
    updateTargetAnimations();

    // Update obstacle animations
    updateObstacleAnimations();

    // Atualizar rotação das hélices continuamente
    for (const [droneId, droneMesh] of drones) {
        const propellers = droneMesh.children.filter(child => child.userData.isPropeller);
        propellers.forEach((prop, index) => {
            if (currentState?.coordinator?.drones?.[droneId]?.is_active) {
                prop.rotation.z += 0.3 + index * 0.05;
            }
        });
    }

    renderer.render(scene, camera);
}

// Event listeners
document.getElementById('start-btn').addEventListener('click', () => {
    sendMessage({
        type: 'command',
        command: 'start'
    });
});

document.getElementById('stop-btn').addEventListener('click', () => {
    sendMessage({
        type: 'command',
        command: 'stop'
    });
});

document.getElementById('reset-btn').addEventListener('click', () => {
    // Limpar trilhas ao resetar
    clearAllTrails();
    
    sendMessage({
        type: 'command',
        command: 'reset'
    });
});

function clearAllTrails() {
    // Limpar todas as trilhas
    for (const [droneId, trailData] of droneTrails) {
        trailData.positions = [];
        trailData.line.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
    }
}

document.getElementById('add-obstacle-btn').addEventListener('click', () => {
    const x = parseFloat(document.getElementById('obs-x').value) || 50;
    const y = parseFloat(document.getElementById('obs-y').value) || 50;
    const z = parseFloat(document.getElementById('obs-z').value) || 10;
    const radius = parseFloat(document.getElementById('obs-radius').value) || 3;

    sendMessage({
        type: 'command',
        command: 'add_obstacle',
        params: { x, y, z, radius }
    });
});

// Formation buttons
document.querySelectorAll('.formation-option').forEach(button => {
    button.addEventListener('click', () => {
        // Remove active class from all buttons
        document.querySelectorAll('.formation-option').forEach(btn => {
            btn.classList.remove('active');
        });

        // Add active class to clicked button
        button.classList.add('active');

        const formation = button.dataset.formation;
        sendMessage({
            type: 'command',
            command: 'set_formation',
            params: { formation }
        });
    });
});

// Window resize
window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// Initialize
init();
