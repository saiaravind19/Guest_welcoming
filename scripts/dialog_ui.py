from PyQt5.QtWidgets import ( QVBoxLayout,QDialog, QDialogButtonBox,QTableWidget,QTableWidgetItem)
import os

class DeleteFaceDialog(QDialog):
    def __init__(self, parent, base_dir):
        super().__init__(parent)
        self.setWindowTitle('Delete Tagged Faces')
        self.setGeometry(100, 100, 300, 200)
        self.base_dir = base_dir
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout(self)

        self.table_widget = QTableWidget()
        self.table_widget.setColumnCount(1)
        self.table_widget.setHorizontalHeaderLabels(['Name'])

        persons = [name for name in os.listdir(self.base_dir) if os.path.isdir(os.path.join(self.base_dir, name))]
        self.table_widget.setRowCount(len(persons))
        for idx, person_name in enumerate(persons):
            item_person_name = QTableWidgetItem(person_name)
            self.table_widget.setItem(idx, 0, item_person_name)

        layout.addWidget(self.table_widget)
        
        self.button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        layout.addWidget(self.button_box)
    
    def get_selected_people(self):
        selected_rows = self.table_widget.selectionModel().selectedRows()
        selected_people = []
        for row in selected_rows:
            if row.column() == 0:  # Ensure we are only interested in the first column
                item = self.table_widget.item(row.row(), 0)
                if item is not None:
                    selected_people.append(item.text())
        return selected_people
    

class ListFacesDialog(QDialog):
    def __init__(self, parent, base_dir):
        super().__init__(parent)
        self.setWindowTitle('Tagged Faces')
        self.setGeometry(100, 100, 300, 200)
        self.base_dir = base_dir
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout(self)

        self.table_widget = QTableWidget()
        self.table_widget.setColumnCount(1)
        self.table_widget.setHorizontalHeaderLabels(['Name'])

        persons = [name for name in os.listdir(self.base_dir) if os.path.isdir(os.path.join(self.base_dir, name))]
        self.table_widget.setRowCount(len(persons))
        for idx, person_name in enumerate(persons):
            item_person_name = QTableWidgetItem(person_name)
            self.table_widget.setItem(idx, 0, item_person_name)

        layout.addWidget(self.table_widget)

        self.button_box = QDialogButtonBox(QDialogButtonBox.Ok)
        self.button_box.accepted.connect(self.accept)
        layout.addWidget(self.button_box)
