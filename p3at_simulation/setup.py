import os
from glob import glob
from setuptools import setup

package_name = 'p3at_simulation'

# --- Início do Bloco de Instalação de Meshes ---
# Lista para armazenar as regras de instalação de arquivos de dados
data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
]

# Lógica para copiar recursivamente a pasta 'meshes'
# Ele vai percorrer cada subpasta dentro de 'meshes'
for dirpath, _, filenames in os.walk('meshes'):
    # Cria uma lista de arquivos de origem (ex: 'meshes/p3dx_meshes/chassis.stl')
    src_files = [os.path.join(dirpath, f) for f in filenames]
    # Cria o caminho de destino correspondente na pasta de instalação
    # (ex: 'share/p3at_simulation/meshes/p3dx_meshes')
    install_path = os.path.join('share', package_name, dirpath)
    # Adiciona a regra à lista de data_files
    data_files.append((install_path, src_files))
# --- Fim do Bloco de Instalação de Meshes ---


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # Usamos a lista que acabamos de preencher
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edson',
    maintainer_email='edson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
              'p3at_controller = p3at_simulation.p3at_controller:main',
        ],
    },
)
