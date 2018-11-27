package TEST;

import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ActionGlyphPainter;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D.PolicyGlyphRenderStyle;
import burlap.mdp.core.state.vardomain.VariableDomain;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.geom.AffineTransform;
import java.awt.geom.Rectangle2D.Float;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.awt.image.ImageObserver;

public class ArrowActionGlyph implements ActionGlyphPainter {
    protected int direction;
    protected Color fillColor;

    public static PolicyGlyphPainter2D getNSEWPolicyGlyphPainter(Object xVar, Object yVar, VariableDomain xRange, VariableDomain yRange, double xWidth, double yWidth, String northActionName, String southActionName, String eastActionName, String westActionName) {
        PolicyGlyphPainter2D spp = new PolicyGlyphPainter2D();
        spp.setXYKeys(xVar, yVar, xRange, yRange, xWidth, yWidth);
        spp.setActionNameGlyphPainter(northActionName, new burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph(0));
        spp.setActionNameGlyphPainter(southActionName, new burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph(1));
        spp.setActionNameGlyphPainter(eastActionName, new burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph(2));
        spp.setActionNameGlyphPainter(westActionName, new burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph(3));
        spp.setRenderStyle(PolicyGlyphRenderStyle.DISTSCALED);
        return spp;
    }

    public ArrowActionGlyph(int direction) {
        this.fillColor = Color.BLACK;
        this.direction = direction;
    }

    public void paintGlyph(Graphics2D g2, float x, float y, float width, float height) {
        int minSize = 30;
        if (width >= (float)minSize && height >= (float)minSize) {
            BufferedImage glyphImage = new BufferedImage((int)width, (int)width, 2);
            Graphics2D img = (Graphics2D)glyphImage.getGraphics();
            float cx = width / 2.0F;
            float arrowHeight = 0.15F * width;
            float shaftWidth = 0.05F * width;
            float shaftHeight = width / 2.0F - arrowHeight;
            float shaftRadius = shaftWidth / 2.0F;
            float sx = cx - shaftRadius;
            img.setColor(this.fillColor);
            img.fill(new Float(sx, arrowHeight, shaftWidth, shaftHeight));
            float arrowHeadWidth = 2.5F * shaftRadius;
            int[] xTriangle = new int[]{(int)(cx - arrowHeadWidth), (int)cx, (int)(cx + arrowHeadWidth)};
            int[] yTriangle = new int[]{(int)arrowHeight, 0, (int)arrowHeight};
            Polygon triangle = new Polygon(xTriangle, yTriangle, 3);
            img.fillPolygon(triangle);
            if (this.direction == 0) {
                g2.drawImage(glyphImage, (int)x, (int)y, (ImageObserver)null);
            } else {
                double locationX = (double)(width / 2.0F);
                double locationY = (double)(width / 2.0F);
                double rotationRequired = 0.0D;
                if (this.direction == 1) {
                    rotationRequired = 3.141592653589793D;
                } else if (this.direction == 2) {
                    rotationRequired = 1.5707963267948966D;
                } else if (this.direction == 3) {
                    rotationRequired = 4.71238898038469D;
                } else if (this.direction == 4) {
                    rotationRequired = 0.785D;
                } else if (this.direction == 5) {
                    rotationRequired = 2.355D;
                } else if (this.direction == 6) {
                    rotationRequired = 5.495D;
                } else if (this.direction == 7) {
                    rotationRequired = 3.925D;
                }

                AffineTransform tx = AffineTransform.getRotateInstance(rotationRequired, locationX, locationY);
                AffineTransformOp op = new AffineTransformOp(tx, 2);
                g2.drawImage(op.filter(glyphImage, (BufferedImage)null), (int)x, (int)y, (ImageObserver)null);
            }

        }
    }
}