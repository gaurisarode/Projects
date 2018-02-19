<%@ taglib uri="http://java.sun.com/jsp/jstl/core" prefix="c" %>
        <%@ taglib uri="http://java.sun.com/jsp/jstl/sql" prefix="sql" %>
        
<sql:query dataSource="${ds}" var="result">
SELECT * from news where category_id = <c:out value="${param.cat }"></c:out> ;
</sql:query>  

<div class="form-group">
                                      <label class="control-label col-lg-2" for="inputSuccess">Select News</label>
                                      <div class="col-lg-10">
                                      
                                          <select class="form-control m-bot15" name="news">
                                          <c:forEach var="row" items="${result.rows}">
                                              <option value="<c:out value="${row.id }"></c:out>"><c:out value="${row.title }"></c:out> <br>
                                              <c:out value="${row.date }"></c:out><hr>
                                              </option>
                                              
                                              </c:forEach>
                                          </select>

                                       
                                      </div>
                                  </div>
                                  
                                   <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Upload File <span class="required">*</span></label>
                                          <div class="col-lg-5">
                                              <input class="form-control" id="cname" name="file"  type="file" required />
                                          </div>
                                      </div>
                                  
                                  <div class="form-group">
                                          <div class="col-lg-offset-2 col-lg-10">
                                              <button class="btn btn-primary" type="submit">Upload</button>
                                              
                                          </div>
                                          </div>
                                          
                                          