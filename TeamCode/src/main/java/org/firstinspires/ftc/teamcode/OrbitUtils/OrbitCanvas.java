    package org.firstinspires.ftc.teamcode.OrbitUtils;
    
    import android.graphics.Bitmap;
    import android.graphics.BitmapFactory;
    import android.graphics.Canvas;
    import android.graphics.Paint;
    import android.graphics.Color;
    
    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
    
    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import org.firstinspires.ftc.teamcode.OrbitHardware.HardwareColors;
    import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;
    import org.firstinspires.ftc.teamcode.robotData.Constants;
    
    public class OrbitCanvas {
        private final int width;
        private final int height;
        private final String name;
        private final Bitmap bitmap;
        private final Canvas canvas;
        private final Paint paint;
    
        public OrbitCanvas(final String name, final int width, final int height) {
            this.width = width;
            this.height = height;
            this.name = name;
            this.bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
            this.canvas = new Canvas(bitmap);
            this.paint = new Paint();
            this.paint.setColor(Color.BLACK);
            this.paint.setStrokeWidth(5);
        }
    
        public Bitmap getBitMap() {
            return bitmap;
        }
    
        public void sendToDashboard(final FtcDashboard dashboard, final TelemetryPacket packet) {
            dashboard.sendTelemetryPacket(packet);
            dashboard.sendImage(bitmap);
        }
    
        public void printToDriverHub(final Telemetry telemetry) {
            telemetry.addData(name, bitmap);
            telemetry.update();
        }
    
        public void addLine(final HardwareColors color, final Vector startPoint, final float radius) {
            paint.setColor(color.toArgb());
            canvas.drawCircle(startPoint.x, startPoint.y, radius, paint);
        }
    
        public void addCircle(final HardwareColors color, final Vector center, final double radius) {
            paint.setColor(color.toArgb());
            canvas.drawCircle(center.x, center.y, (float) radius, paint);
        }
    
        public void addRectangle(final HardwareColors color, final Vector start, final Vector end) {
            paint.setColor(color.toArgb());
            canvas.drawRect(start.x, start.y, end.x, end.y, paint);
        }
    
        public void addArrow(final HardwareColors color, final Vector position, final float size, final float heading) {
            paint.setColor(color.toArgb());
            paint.setStrokeWidth(5);
    
            float endX = (float) (position.x + size * Math.cos(heading));
            float endY = (float) (position.y + size * Math.sin(heading));
    
            canvas.drawLine(position.x, position.y, endX, endY, paint);
    
            float arrowHeadSize = size / 4;
            float arrowAngle = (float) Math.PI / 6;
    
            float leftX = (float) (endX - arrowHeadSize * Math.cos(heading - arrowAngle));
            float leftY = (float) (endY - arrowHeadSize * Math.sin(heading - arrowAngle));
    
            float rightX = (float) (endX - arrowHeadSize * Math.cos(heading + arrowAngle));
            float rightY = (float) (endY - arrowHeadSize * Math.sin(heading + arrowAngle));
    
            canvas.drawLine(endX, endY, leftX, leftY, paint);
            canvas.drawLine(endX, endY, rightX, rightY, paint);
        }
    
    
        public void addText(final HardwareColors color, final Vector position, final String text) {
            paint.setColor(color.toArgb());
            canvas.drawText(text, position.x, position.y, paint);
        }
    
        public void addImageFromPath(final String imagePath, final Vector position) {
            final Bitmap imageBitmap = BitmapFactory.decodeFile(imagePath);
            if (imageBitmap != null) {
                canvas.drawBitmap(imageBitmap, position.x, position.y, paint);
            }
        }
    
        public void drawRobot(final HardwareColors color, final Pose2D pose) {
            addCircle(color, pose.translation, Constants.robotRadios);
            addArrow(HardwareColors.BLACK, pose.translation, Constants.robotRadios - 5, pose.rotation);
        }
    
        public void drawRobot(final HardwareColors color) {
            final Pose2D pose = OrbitPoseTracker.getRobotOrbitPose2D();
            addCircle(color, pose.translation, Constants.robotRadios);
            addArrow(HardwareColors.BLACK, pose.translation, Constants.robotRadios - 5, pose.rotation);
        }
    
        public void clearCanvas() {
            canvas.drawColor(Color.WHITE);
        }
    
        public void removeLine(final Vector startPoint, final float radius) {
            paint.setColor(Color.WHITE);
            canvas.drawCircle(startPoint.x, startPoint.y, radius, paint);
        }
    
        public void removeCircle(final Vector center, final double radius) {
            paint.setColor(Color.WHITE);
            canvas.drawCircle(center.x, center.y, (float) radius, paint);
        }
    
        public void removeRectangle(final Vector start, final Vector end) {
            paint.setColor(Color.WHITE);
            canvas.drawRect(start.x, start.y, end.x, end.y, paint);
        }
    
        public void removeArrow(final Vector position, final float size, final float heading) {
            paint.setColor(Color.WHITE);
            paint.setStrokeWidth(5);
    
            float endX = (float) (position.x + size * Math.cos(heading));
            float endY = (float) (position.y + size * Math.sin(heading));
    
            canvas.drawLine(position.x, position.y, endX, endY, paint);
    
            float arrowHeadSize = size / 4;
            float arrowAngle = (float) Math.PI / 6;
    
            float leftX = (float) (endX - arrowHeadSize * Math.cos(heading - arrowAngle));
            float leftY = (float) (endY - arrowHeadSize * Math.sin(heading - arrowAngle));
    
            float rightX = (float) (endX - arrowHeadSize * Math.cos(heading + arrowAngle));
            float rightY = (float) (endY - arrowHeadSize * Math.sin(heading + arrowAngle));
    
            canvas.drawLine(endX, endY, leftX, leftY, paint);
            canvas.drawLine(endX, endY, rightX, rightY, paint);
        }
    
        public void removeText(final Vector position, final String text) {
            paint.setColor(Color.WHITE);
            canvas.drawText(text, position.x, position.y, paint);
        }
    
        public void removeImageFromPath(final String imagePath, final Vector position) {
            final Bitmap imageBitmap = BitmapFactory.decodeFile(imagePath);
            if (imageBitmap != null) {
                paint.setColor(Color.WHITE);
                canvas.drawBitmap(imageBitmap, position.x, position.y, paint);
            }
        }
    
        public void removeRobot(final Pose2D pose) {
            removeCircle(pose.translation, Constants.robotRadios);
            removeArrow(pose.translation, Constants.robotRadios - 5, pose.rotation);
        }
    
        public void removeRobot() {
            final Pose2D pose = OrbitPoseTracker.getRobotOrbitPose2D();
            removeCircle(pose.translation, Constants.robotRadios);
            removeArrow(pose.translation, Constants.robotRadios - 5, pose.rotation);
        }
    
        public void setField(final String fieldPath) {
            final Bitmap fieldBitmap = BitmapFactory.decodeFile(fieldPath);
            if (fieldBitmap != null) {
                canvas.drawBitmap(fieldBitmap, 0, 0, paint);
            }
        }
    }