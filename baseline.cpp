/* 
ESP-WHO(ESP32 기반의 얼굴 감지 및 인식 프레임워크)
https://www.joyk.com/dig/detail/1540675811437287
*/


#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "fd_forward.h"  // 얼굴 감지
#include "fr_forward.h"  // 얼굴 인식
#include "WiFi.h"
#include "WebServer.h"

// 카메라 핀 설정 (ESP32-CAM AI-Thinker)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Wi-Fi 설정
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// 도어락 제어 핀
#define RELAY_PIN 12
#define LED_PIN 33
#define BUZZER_PIN 13

// 웹서버 객체
WebServer server(80);

// 얼굴 인식 관련 변수
static mtmn_config_t mtmn_config = mtmn_init_config();
static face_id_list id_list = {0};
bool face_recognition_enabled = true;

void setup() {
  Serial.begin(115200);
  
  // GPIO 초기화
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  digitalWrite(RELAY_PIN, LOW);  // 도어락 잠금
  digitalWrite(LED_PIN, LOW);
  
  // 카메라 초기화
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // 메모리에 따른 해상도 설정
  if(psramFound()) {
    config.frame_size = FRAMESIZE_QVGA; // 320x240
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // 카메라 초기화
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("카메라 초기화 실패: 0x%x", err);
    return;
  }
  
  // Wi-Fi 연결
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi 연결됨");
  Serial.print("IP 주소: ");
  Serial.println(WiFi.localIP());
  
  // 웹서버 라우팅 설정
  server.on("/", handle_root);
  server.on("/capture", handle_capture);
  server.on("/enroll", handle_enroll);
  server.on("/recognize", handle_recognize);
  server.on("/delete", handle_delete);
  server.on("/unlock", handle_unlock);
  
  server.begin();
  Serial.println("웹서버 시작됨");
  
  // LED 점등 (시스템 준비 완료)
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  server.handleClient();
  
  if (face_recognition_enabled) {
    performFaceRecognition();
  }
  
  delay(100);
}

void performFaceRecognition() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("카메라 캡처 실패");
    return;
  }
  
  // 이미지를 RGB565로 변환
  uint8_t* rgb565_buf = NULL;
  bool converted = fmt2rgb565(fb->buf, fb->len, fb->format, &rgb565_buf);
  
  if (converted) {
    // 얼굴 감지 수행
    dl_matrix3du_t* image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
    fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
    
    // MTMN으로 얼굴 감지
    box_array_t* net_boxes = face_detect(image_matrix, &mtmn_config);
    
    if (net_boxes) {
      // 얼굴이 감지된 경우
      Serial.printf("감지된 얼굴 수: %d\n", net_boxes->len);
      
      for (int i = 0; i < net_boxes->len; i++) {
        // 얼굴 인식 수행
        int8_t face_id = run_face_recognition(image_matrix, net_boxes, i);
        
        if (face_id >= 0) {
          // 등록된 얼굴 인식됨
          Serial.printf("얼굴 인식 성공! ID: %d\n", face_id);
          unlockDoor();
        } else {
          // 미등록 얼굴
          Serial.println("미등록 얼굴입니다.");
          playBuzzer(2); // 경고음
        }
      }
      
      dl_lib_free(net_boxes->score);
      dl_lib_free(net_boxes->box);
      dl_lib_free(net_boxes->landmark);
      dl_lib_free(net_boxes);
    }
    
    dl_matrix3du_free(image_matrix);
    free(rgb565_buf);
  }
  
  esp_camera_fb_return(fb);
}

int8_t run_face_recognition(dl_matrix3du_t* image_matrix, box_array_t* net_boxes, int face_index) {
  dl_matrix3du_t* aligned_face = NULL;
  int8_t face_id = -1;
  
  // 얼굴 정렬
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  if (align_face(net_boxes, image_matrix, aligned_face, face_index) == ESP_OK) {
    // 얼굴 인식 수행
    face_id = recognize_face(&id_list, aligned_face);
  }
  
  if (aligned_face) {
    dl_matrix3du_free(aligned_face);
  }
  
  return face_id;
}

void unlockDoor() {
  Serial.println("문 열림!");
  digitalWrite(RELAY_PIN, HIGH); // 릴레이 활성화 (도어락 해제)
  digitalWrite(LED_PIN, HIGH);   // LED 점등
  playBuzzer(1);                 // 성공음
  
  delay(3000); // 3초간 열림 상태 유지
  
  digitalWrite(RELAY_PIN, LOW);  // 도어락 다시 잠금
  digitalWrite(LED_PIN, LOW);
  Serial.println("문 잠김");
}

void playBuzzer(int pattern) {
  switch(pattern) {
    case 1: // 성공음
      digitalWrite(BUZZER_PIN, HIGH);
      delay(200);
      digitalWrite(BUZZER_PIN, LOW);
      break;
    case 2: // 경고음
      for(int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(100);
        digitalWrite(BUZZER_PIN, LOW);
        delay(100);
      }
      break;
  }
}

// 웹 인터페이스 핸들러들
void handle_root() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<title>ESP32-CAM 얼굴인식 도어락</title></head><body>";
  html += "<h1>얼굴인식 도어락 시스템</h1>";
  html += "<h2>등록된 얼굴 ID 수: " + String(id_list.count) + "</h2>";
  html += "<img src='/capture' style='width:400px;'><br><br>";
  html += "<button onclick=\"location.href='/enroll'\">새 얼굴 등록</button><br><br>";
  html += "<button onclick=\"location.href='/recognize'\">얼굴 인식 테스트</button><br><br>";
  html += "<button onclick=\"location.href='/delete'\">마지막 등록 삭제</button><br><br>";
  html += "<button onclick=\"location.href='/unlock'\">수동 문열기</button><br><br>";
  html += "<script>setInterval(function(){document.querySelector('img').src='/capture?'+Date.now()}, 2000);</script>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void handle_capture() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "카메라 오류");
    return;
  }
  
  server.sendHeader("Content-Type", "image/jpeg");
  server.sendHeader("Content-Length", String(fb->len));
  server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
  
  esp_camera_fb_return(fb);
}

void handle_enroll() {
  face_recognition_enabled = false; // 등록 중에는 인식 중지
  
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "카메라 오류");
    return;
  }
  
  // 이미지 변환 및 얼굴 감지
  dl_matrix3du_t* image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
  
  box_array_t* net_boxes = face_detect(image_matrix, &mtmn_config);
  
  if (net_boxes && net_boxes->len > 0) {
    // 첫 번째 감지된 얼굴을 등록
    dl_matrix3du_t* aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
    
    if (align_face(net_boxes, image_matrix, aligned_face, 0) == ESP_OK) {
      int8_t face_id = enroll_face(&id_list, aligned_face);
      
      if (face_id >= 0) {
        server.send(200, "text/plain", "얼굴 등록 성공! ID: " + String(face_id));
        Serial.printf("새 얼굴 등록됨. ID: %d\n", face_id);
      } else {
        server.send(500, "text/plain", "얼굴 등록 실패");
      }
    } else {
      server.send(500, "text/plain", "얼굴 정렬 실패");
    }
    
    dl_matrix3du_free(aligned_face);
    dl_lib_free(net_boxes->score);
    dl_lib_free(net_boxes->box);
    dl_lib_free(net_boxes->landmark);
    dl_lib_free(net_boxes);
  } else {
    server.send(500, "text/plain", "얼굴이 감지되지 않았습니다");
  }
  
  dl_matrix3du_free(image_matrix);
  esp_camera_fb_return(fb);
  
  face_recognition_enabled = true; // 인식 재개
}

void handle_recognize() {
  server.send(200, "text/plain", "얼굴 인식 테스트 시작");
  performFaceRecognition();
}

void handle_delete() {
  int8_t deleted_id = delete_face(&id_list);
  if (deleted_id >= 0) {
    server.send(200, "text/plain", "얼굴 ID " + String(deleted_id) + " 삭제됨");
  } else {
    server.send(500, "text/plain", "삭제할 얼굴이 없습니다");
  }
}

void handle_unlock() {
  unlockDoor();
  server.send(200, "text/plain", "수동으로 문을 열었습니다");
}
