package org.firstinspires.ftc.teamcode.OrbitUtils.OrbitLogger;

import android.os.Environment;

import org.firstinspires.ftc.teamcode.OrbitUtils.Feedback;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;



public final class CSVWriter {

    private final Feedback<Float[]> data;
    private final String fileName;
    private final int lookBackCycles;
    private final String titles;
    private final Boolean addMatchNumber;

    private final ArrayList<Float> currentLine = new ArrayList<>();

    private boolean hasDataToSave = false;

    public CSVWriter(final String fileName, final int lookBackCycles, final String titles,
                     final boolean addMatchNumber) {
        this.data = new Feedback<Float[]>(new Float[] {}, lookBackCycles);
        this.fileName = fileName + (GlobalData.inAutonomous ? "auto" : "teleop");
        this.lookBackCycles = lookBackCycles;
        this.titles = titles;
        this.addMatchNumber = addMatchNumber;

    }

    private void renameFile() {
        final String fileName = addMatchNumber ? "Log" + "0" : this.fileName;
        final File file = new File((String.format("%s/FIRST/" + fileName + ".csv",
                Environment.getExternalStorageDirectory().getAbsolutePath())));

        if (file.exists()) {
            final File oldFile = new File((String.format("%s/FIRST/" + fileName + ".csv",
                    Environment.getExternalStorageDirectory().getAbsolutePath())));
            oldFile.delete();

            file.renameTo(oldFile);
        }

        try {
            final String oldFileName = addMatchNumber ? "Log" + "old" : this.fileName;
            final File usbFile = new File((String.format("%s/FIRST/" + oldFileName + ".csv",
                    Environment.getExternalStorageDirectory().getAbsolutePath())));;

            if (usbFile.exists()) {
                final File oldUsbFile = new File((String.format("%s/FIRST/" + fileName + ".csv",
                        Environment.getExternalStorageDirectory().getAbsolutePath())));
                oldUsbFile.delete();

                usbFile.renameTo(oldUsbFile);
            }
        } catch (Exception e) {
            System.out.println("No USB connected. Can't rename file :(");
        }
    }

    public void addData(final Float[] line) {
        this.data.update(line);

        hasDataToSave = true;
    }

    public void addDataToLine(final float... data) {
        for (final float x : data)
            currentLine.add(x);
        hasDataToSave = true;
    }

    public void endLine() {
        final Float[] currentLine = this.currentLine.toArray(new Float[this.currentLine.size()]);
        this.data.update(currentLine);
        this.currentLine.clear();
    }

    public void saveFile() {
        System.out.println("saving");

        if (!hasDataToSave) {
            System.out.println("didnt save");
            return;
        }
        this.renameFile();

        try {
            final String fileName = (addMatchNumber ? "Log"  : this.fileName) + " " + (GlobalData.inAutonomous ? "auto" : "teleop");
            final File file = new File((String.format("%s/FIRST/" + fileName + ".csv",
                    Environment.getExternalStorageDirectory().getAbsolutePath())));
            final PrintWriter writer = new PrintWriter(file);

            // Write titles
            writer.print(this.titles);
            writer.print("\n");

            // Write values
            for (int i = lookBackCycles; i >= 0; i--) {
                if (this.data.getFeedback(i).length == 0) {
                    continue;
                }

                for (final float value : this.data.getFeedback(i)) {
                    writer.print(value + ",");
                }
                writer.print("\n");
            }

            writer.close();
            System.out.println("Saved successfully on control hub. :)");

        } catch (final FileNotFoundException e) {
            System.out.println("CSV logger file not found. :(");
        }

        try {
            final String fileName = addMatchNumber ? this.fileName + "0" : this.fileName;
            final File usbFile = new File("/U/" + fileName + ".csv");
            final PrintWriter usbWriter = new PrintWriter(usbFile);

            // Write titles
            usbWriter.print(this.titles);
            usbWriter.print("\n");

            // Write values
            for (int i = lookBackCycles; i >= 0; i--) {
                if (this.data.getFeedback(i).length == 0) {
                    continue;
                }

                for (final float value : this.data.getFeedback(i)) {
                    usbWriter.print(value + ",");
                }
                usbWriter.print("\n");
            }

            usbWriter.close();
            System.out.println("Saved successfully on USB. :)");

        } catch (final FileNotFoundException e) {
            System.out.println("CSV logger USB file not found. :(");
        }

        hasDataToSave = false;
    }
}