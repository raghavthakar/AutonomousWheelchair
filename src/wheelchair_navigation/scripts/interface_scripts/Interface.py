import PySimpleGUI as sg
import os.path
import csv

import MapLoader

# First the window layout in 2 columns
file_list_column = [
    [
        sg.Text("Image Folder"),
        sg.In(size=(25, 1), enable_events=True, key="-FOLDER-"),
        sg.FolderBrowse(),
    ],
    [
        sg.Listbox(
            values=[], enable_events=True, size=(40, 20), key="-FILE LIST-"
        )
    ],
]

# For now will only show the name of the file that was chosen
image_viewer_column = [
    [sg.Text("Select the image of the map to open, then place start and target points. After placing a point, press any key to close the map.", size=(53, 2))],
    [sg.Button("Starting Point"), sg.Button("Target Point")],
    [sg.HSeparator()],
    [sg.Text("Choose an image from list on left:")],
    [sg.Text(size=(40, 2), key="-TOUT-")],
    [sg.HSeparator()],
    [sg.Text("Starting point's coordinates:")],
    [sg.Text(size=(40, 2), key="-START-")],
    [sg.Text("Target point's coordinates:")],
    [sg.Text(size=(40, 2), key="-TARGET-")],
    [sg.HSeparator()],
    [sg.Text("Press 'Validate' to save coordinates in the csv file")],
    [sg.Button("Validate", button_color=('white', 'green')), sg.Push(), sg.Button("Close", button_color=('white', 'firebrick3'))],
]

# ----- Full layout -----
layout = [
    [
        sg.Column(file_list_column),
        sg.VSeperator(),
        sg.Column(image_viewer_column),
    ]
]

window = sg.Window("Map Viewer", layout)

# Run the Event Loop
while True:

    event, values = window.read()

    # break the loop if the window is closed
    if event == "Close" or event == sg.WIN_CLOSED:
        break

    # Folder name was filled in, make a list of files in the folder
    if event == "-FOLDER-":

        folder = values["-FOLDER-"]

        try:
            # Get list of files in folder
            file_list = os.listdir(folder)

        except:
            file_list = []

        fnames = [
            f
            for f in file_list
            if os.path.isfile(os.path.join(folder, f))
            and f.lower().endswith((".png", "jpg")) # take png and jpg files
        ]
        window["-FILE LIST-"].update(fnames)

    elif event == "Starting Point":  # The user want to place the starting point
        try:

            filename = os.path.join(values["-FOLDER-"], values["-FILE LIST-"][0]) # open the selected file in the list
            window["-TOUT-"].update(filename)
            
            CoordinatesS = MapLoader.LoadMap(filename) # Updatde starting coordinates
            window["-START-"].update(CoordinatesS)
            print(CoordinatesS)
        except:
            pass

    elif event == "Target Point":  # The user want to place the target point
        try:

            filename = os.path.join(values["-FOLDER-"], values["-FILE LIST-"][0]) 
            window["-TOUT-"].update(filename)
            
            CoordinatesT = MapLoader.LoadMap(filename) # Updatde targeted coordinates
            window["-TARGET-"].update(CoordinatesT)

        except:
            pass
    
    elif event == "Validate":
        try:
            with open('coordinates.csv', 'w', newline='') as csvfile: # Open the csv file to write in
                writer = csv.DictWriter(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL, fieldnames=['Starting coordinates', 'Targeted coordinates'])
                writer.writeheader() # Write the header for both columns
                writer.writerow({'Starting coordinates': CoordinatesS, 'Targeted coordinates': CoordinatesT}) # Write the data for both columns

        except:
            pass

window.close()