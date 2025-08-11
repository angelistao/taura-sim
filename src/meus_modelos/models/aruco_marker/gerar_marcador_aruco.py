import cv2
import numpy as np
import os

# --- Configurações ---
# Dicionário ArUco. Tem que ser o mesmo usado no seu nó de detecção ROS!
# Exemplos: cv2.aruco.DICT_4X4_100, cv2.aruco.DICT_6X6_250, etc.
ARUCO_DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

# ID do marcador que você quer gerar
MARKER_ID = 23

# Tamanho da imagem em pixels
PIXEL_SIZE = 500

# Nome do arquivo de saída
OUTPUT_FILENAME = f"marker_{MARKER_ID}.png"

# --- Geração do Marcador ---
print(f"Gerando marcador ArUco...")
print(f"  - Dicionário: {ARUCO_DICTIONARY.bytesList.shape}")
print(f"  - ID: {MARKER_ID}")
print(f"  - Tamanho: {PIXEL_SIZE}x{PIXEL_SIZE} pixels")

# Aloca memória para a imagem do marcador
marker_image = np.zeros((PIXEL_SIZE, PIXEL_SIZE), dtype=np.uint8)

# Gera a imagem do marcador ArUco
# O '1' no final é a espessura da borda (borderBits)
marker_image = cv2.aruco.generateImageMarker(ARUCO_DICTIONARY, MARKER_ID, PIXEL_SIZE, marker_image, 1)

# Salva a imagem no disco
cv2.imwrite(OUTPUT_FILENAME, marker_image)

print(f"\nMarcador salvo como '{OUTPUT_FILENAME}' no diretório atual.")
print("Use esta imagem como uma textura no Gazebo.")

# Opcional: Mostra a imagem gerada
# cv2.imshow("Marcador Gerado", marker_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()