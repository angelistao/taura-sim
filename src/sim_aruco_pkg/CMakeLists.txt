cmake_minimum_required(VERSION 3.8)
project(sim_aruco_pkg)

# Encontra as dependências do ament
find_package(ament_cmake REQUIRED)

# Instala o package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Instala o seu diretório de launch
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Instala o seu diretório de mundos do Gazebo
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Instala o seu diretório de scripts Python
install(
  DIRECTORY scripts
  DESTINATION lib/${PROJECT_NAME}
)

# Adiciona o diretório de modelos do Gazebo ao path
# para que o Gazebo possa encontrar "model://aruco_marker"
install(
  DIRECTORY ../meus_modelos/models/aruco_marker
  DESTINATION share/${PROJECT_NAME}/models
)

# Exporta o path dos modelos para o Gazebo
set(GAZEBO_MODEL_PATH_EXPORT "share/${PROJECT_NAME}/models")
ament_environment_hooks(
  "ament_prefix_path/local_setup.bash"
  "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${ament_prefix_path}/${GAZEBO_MODEL_PATH_EXPORT}"
)


ament_package()# Instala o diretório de materiais do Gazebo