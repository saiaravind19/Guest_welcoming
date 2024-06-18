from PyQt5.QtWidgets import (QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget,
                             QPushButton, QTextEdit, QInputDialog, QDialog)
from PyQt5.QtGui import QPixmap
from dialog_ui import ListFacesDialog,DeleteFaceDialog
from face_detection import VideoThread
import os

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("PyQt OpenCV Camera Feed with Face Detection")
        self.setGeometry(100, 100, 800, 600)

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        self.image_label = QLabel(self)
        self.image_label.resize(640, 480)

        self.tag_face_button = QPushButton("Tag Face", self)
        self.tag_face_button.clicked.connect(self.tag_face)

        self.delete_face_button = QPushButton("Delete Tagged Faces", self)
        self.delete_face_button.clicked.connect(self.delete_tagged_faces)

        self.list_faces_button = QPushButton("List Tagged Faces", self)
        self.list_faces_button.clicked.connect(self.list_tagged_faces)

        self.log_text = QTextEdit(self)
        self.log_text.setReadOnly(True)
        self.log_text.setFixedHeight(100)

        button_layout = QVBoxLayout()
        button_layout.addWidget(self.tag_face_button)
        button_layout.addWidget(self.delete_face_button)
        button_layout.addWidget(self.list_faces_button)
        button_layout.addStretch()

        image_button_layout = QHBoxLayout()
        image_button_layout.addWidget(self.image_label)
        image_button_layout.addLayout(button_layout)

        main_layout = QVBoxLayout()
        main_layout.addLayout(image_button_layout)
        main_layout.addWidget(self.log_text)

        self.central_widget.setLayout(main_layout)

        self.thread = VideoThread()
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.start()

    def closeEvent(self, event):
        self.thread.stop()
        event.accept()

    def update_image(self, cv_img):
        self.image_label.setPixmap(QPixmap.fromImage(cv_img))

    def tag_face(self):
        text, ok = QInputDialog.getText(self, 'Tag Face', 'Enter name for tagging:')
        if ok and text:
            text = text.strip()  # Remove leading and trailing whitespace
            self.log_text.append(f"Tagging user: {text}")
            self.thread.start_tagging(text)


    
    def delete_tagged_faces(self):
        dialog = DeleteFaceDialog(self, self.thread.base_dir)
        if dialog.exec_() == QDialog.Accepted:
            selected_people = dialog.get_selected_people()  # Assuming get_selected_people() returns a list of selected people
            if selected_people:
                for person in selected_people:
                    person_dir = os.path.join(self.thread.base_dir, person)
                    if os.path.exists(person_dir):
                        for root, dirs, files in os.walk(person_dir, topdown=False):
                            for file in files:
                                os.remove(os.path.join(root, file))
                            for dir in dirs:
                                os.rmdir(os.path.join(root, dir))
                        os.rmdir(person_dir)
                        self.log_text.append(f"Deleted tagged faces for {person}")
                    else:
                        self.log_text.append(f"No tagged faces found for {person}")
            else:
                self.log_text.append("No users selected for deletion")

    def list_tagged_faces(self):
        dialog = ListFacesDialog(self, self.thread.base_dir)
        dialog.exec_()
     