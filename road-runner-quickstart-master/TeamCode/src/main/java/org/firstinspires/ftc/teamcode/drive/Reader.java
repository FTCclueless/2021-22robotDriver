package org.firstinspires.ftc.teamcode.drive;

import android.os.Environment;

import java.io.File;
import java.io.IOException;
import java.util.Scanner;

public class Reader {

    private static final String BASE_FOLDER_NAME = "FIRST";
    private Scanner fileReader;

    public String readFile(String filename){
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME+"/"+filename+".csv";
        File file = new File(directoryPath);
        try {
            fileReader = new Scanner(file);
            String data = "";
            while (fileReader.hasNextLine()) {
                data += fileReader.nextLine();
            }
            return data;
        } catch (IOException e){
            e.printStackTrace();
        }
        return "FileReader Failed";
    }
}
